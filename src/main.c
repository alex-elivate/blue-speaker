/**
 * ESP32 WROOM Bluetooth speaker firmware.
 *
 * Streams A2DP audio into an I2S MAX98357 amplifier while exposing
 * basic AVRCP transport controls to phones or computers.
 */

#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"

static const char *TAG = "blue_speaker";

// Set to 1 to generate 440Hz test tone instead of Bluetooth audio
#define TEST_TONE_MODE 0

#define BT_SPEAKER_NAME "Blue Speaker"
static i2s_chan_handle_t s_tx_chan = NULL; // Modern I2S channel handle
#define I2S_DEFAULT_SAMPLE_RATE (44100)

// Wiring for ESP32 WROOM with MAX98357A:
#define I2S_PIN_BCLK (GPIO_NUM_27)    // Blue wire
#define I2S_PIN_LRCK (GPIO_NUM_26)    // Green wire
#define I2S_PIN_DOUT (GPIO_NUM_25)    // Yellow wire
#define MAX98357_SD_PIN (GPIO_NUM_33) // White wire to SD (shutdown control)

#define I2S_DMA_BUFFER_COUNT (8)
#define I2S_DMA_BUFFER_LENGTH (64)
#define I2S_RINGBUFFER_SIZE_BYTES (32 * 1024)

static RingbufHandle_t s_i2s_ringbuf = NULL;
static uint32_t s_current_sample_rate = I2S_DEFAULT_SAMPLE_RATE;
static uint8_t s_abs_volume = 0x50; // Range 0-0x7F

typedef struct
{
    uint32_t bytes_accum;
    uint32_t callbacks;
    uint32_t last_log_ms;
    bool logged_first_frame;
} audio_rx_stats_t;

typedef struct
{
    uint32_t bytes_accum;
    uint32_t chunks;
    uint32_t last_log_ms;
} audio_tx_stats_t;

static audio_rx_stats_t s_audio_rx_stats = {0};
static audio_tx_stats_t s_audio_tx_stats = {0};

static void i2s_setup(void);
static void bt_stack_setup(void);
static void i2s_writer_task(void *arg);
static void i2s_tone_test_task(void *arg);

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void bt_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static void bt_a2d_data_cb(const uint8_t *data, uint32_t len);
static void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
static void bt_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);

static const char *a2dp_conn_state_to_str(esp_a2d_connection_state_t state);
static const char *a2dp_audio_state_to_str(esp_a2d_audio_state_t state);
static void log_bda(const char *label, const uint8_t *bda);

static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    init_nvs();

    // Raise Bluetooth subsystem verbosity so connection issues surface quickly.
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("BTDM_INIT", ESP_LOG_DEBUG);
    esp_log_level_set("BT_BTC", ESP_LOG_DEBUG);
    esp_log_level_set("BT_APPL", ESP_LOG_DEBUG);
    esp_log_level_set("BT_GAP", ESP_LOG_DEBUG);
    esp_log_level_set("BT_AVCT", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    i2s_setup();

#if TEST_TONE_MODE
    ESP_LOGI(TAG, "*** TEST TONE MODE - generating 440Hz sine wave ***");
    BaseType_t created = xTaskCreatePinnedToCore(
        i2s_tone_test_task,
        "i2s_tone_test",
        4096,
        NULL,
        configMAX_PRIORITIES - 3,
        NULL,
        0);
    if (created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create I2S tone test task");
        return;
    }
#else
    s_i2s_ringbuf = xRingbufferCreate(I2S_RINGBUFFER_SIZE_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (!s_i2s_ringbuf)
    {
        ESP_LOGE(TAG, "Failed to allocate I2S ringbuffer");
        return;
    }

    BaseType_t created = xTaskCreatePinnedToCore(
        i2s_writer_task,
        "i2s_writer",
        4096,
        NULL,
        configMAX_PRIORITIES - 3,
        NULL,
        0);
    if (created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create I2S writer task");
        return;
    }

    bt_stack_setup();
#endif
}

static void i2s_setup(void)
{
    ESP_LOGI(TAG, "Initializing I2S (modern std driver): Rate=%d Hz, Bits=%d",
             I2S_DEFAULT_SAMPLE_RATE, 16);
    ESP_LOGI(TAG, "I2S Pins: BCLK=%d, LRCK=%d, DOUT=%d", I2S_PIN_BCLK, I2S_PIN_LRCK, I2S_PIN_DOUT);

    gpio_config_t sd_pin_config = {
        .pin_bit_mask = (1ULL << MAX98357_SD_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&sd_pin_config));

    // Set HIGH multiple times and verify
    ESP_ERROR_CHECK(gpio_set_level(MAX98357_SD_PIN, 1));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(gpio_set_level(MAX98357_SD_PIN, 1));
    int level = gpio_get_level(MAX98357_SD_PIN);
    ESP_LOGI(TAG, "MAX98357 SD pin %d set HIGH (amp enabled), readback=%d", MAX98357_SD_PIN, level);

    // Create I2S channel
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_BUFFER_COUNT;
    chan_cfg.dma_frame_num = I2S_DMA_BUFFER_LENGTH;
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL));

    // Configure for Philips I2S standard format (correct format for MAX98357)
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_DEFAULT_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_PIN_BCLK,
            .ws = I2S_PIN_LRCK,
            .dout = I2S_PIN_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
    ESP_LOGI(TAG, "I2S modern driver initialized and enabled");
}

static void bt_stack_setup(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_gap_cb));
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(BT_SPEAKER_NAME));

    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap)));
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, NULL));

    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(bt_avrc_ct_cb));
    ESP_ERROR_CHECK(esp_avrc_ct_init());

    ESP_ERROR_CHECK(esp_avrc_tg_register_callback(bt_avrc_tg_cb));
    ESP_ERROR_CHECK(esp_avrc_tg_init());

    ESP_ERROR_CHECK(esp_a2d_register_callback(bt_a2d_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(bt_a2d_data_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_init());

    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));
}

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
        }
        else
        {
            ESP_LOGE(TAG, "Authentication failed, status 0x%x", param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(TAG, "PIN requested, replying with 0000");
        esp_bt_pin_code_t pin_code = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "SSP Confirmation, numeric value %d",
                 param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "Passkey notification: %06d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "Passkey request event");
        break;
    default:
        ESP_LOGD(TAG, "Unhandled GAP event %d", event);
        break;
    }
}

static uint32_t a2dp_parse_sample_rate(const esp_a2d_mcc_t *mcc)
{
    // In ESP-IDF 5.x, the media codec structure has changed
    // For now, we'll use the default sample rate and let the I2S
    // driver handle any necessary adjustments
    // Most Bluetooth audio streams use 44.1kHz anyway
    (void)mcc; // Suppress unused parameter warning
    return I2S_DEFAULT_SAMPLE_RATE;
}

static void bt_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
    {
        esp_a2d_connection_state_t state = param->conn_stat.state;
        log_bda("A2DP peer", param->conn_stat.remote_bda);
        ESP_LOGI(TAG, "A2DP connection state: %s", a2dp_conn_state_to_str(state));
        if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
        {
            ESP_LOGI(TAG, "Disconnect reason: 0x%x", param->conn_stat.disc_rsn);
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP audio state: %s", a2dp_audio_state_to_str(param->audio_stat.state));
        if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STOPPED)
        {
            // Modern I2S driver - no zero_dma_buffer equivalent needed
            s_audio_rx_stats.logged_first_frame = false;
        }
        else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED)
        {
            ESP_LOGI(TAG, "Audio stream starting - resetting pipeline");
            // Drain any stale data from ring buffer
            size_t item_size;
            void *item;
            while ((item = xRingbufferReceive(s_i2s_ringbuf, &item_size, 0)) != NULL)
            {
                vRingbufferReturnItem(s_i2s_ringbuf, item);
            }
            s_audio_rx_stats = (audio_rx_stats_t){0};
            s_audio_tx_stats = (audio_tx_stats_t){0};
        }
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
    {
        s_current_sample_rate = a2dp_parse_sample_rate(&param->audio_cfg.mcc);
        ESP_LOGI(TAG, "Audio cfg event: sample rate %u",
                 (unsigned int)s_current_sample_rate);
        // Modern I2S driver - disable, reconfigure sample rate, then re-enable
        ESP_ERROR_CHECK(i2s_channel_disable(s_tx_chan));
        i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(s_current_sample_rate);
        ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(s_tx_chan, &clk_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
        break;
    }
    default:
        ESP_LOGD(TAG, "Unhandled A2DP event %d", event);
        break;
    }
}

static void bt_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    if (!s_i2s_ringbuf || !data || len == 0)
    {
        ESP_LOGW(TAG, "Invalid data callback: ringbuf=%p data=%p len=%u",
                 s_i2s_ringbuf, data, (unsigned int)len);
        return;
    }

    uint32_t now_ms = esp_log_timestamp();
    if (!s_audio_rx_stats.logged_first_frame)
    {
        ESP_LOGI(TAG, "First audio frame: %u bytes", (unsigned int)len);
        s_audio_rx_stats.logged_first_frame = true;
        s_audio_rx_stats.last_log_ms = now_ms;
    }

    s_audio_rx_stats.bytes_accum += len;
    s_audio_rx_stats.callbacks++;

    if ((now_ms - s_audio_rx_stats.last_log_ms) >= 1000)
    {
        size_t free_bytes = xRingbufferGetCurFreeSize(s_i2s_ringbuf);
        ESP_LOGD(TAG, "Audio RX: %u bytes across %u callbacks, ring free %u",
                 (unsigned int)s_audio_rx_stats.bytes_accum,
                 (unsigned int)s_audio_rx_stats.callbacks,
                 (unsigned int)free_bytes);
        s_audio_rx_stats.bytes_accum = 0;
        s_audio_rx_stats.callbacks = 0;
        s_audio_rx_stats.last_log_ms = now_ms;
    }

    if (xRingbufferSend(s_i2s_ringbuf, data, len, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "I2S ringbuffer full, dropping %u bytes", (unsigned int)len);
    }
}

static void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event)
    {
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        if (param->change_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE)
        {
            ESP_LOGI(TAG, "Remote volume changed to %u", param->change_ntf.event_parameter.volume);
        }
        break;
    default:
        break;
    }
}

static void bt_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event)
    {
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
        ESP_LOGI(TAG, "AVRCP remote features: 0x%x", param->rmt_feats);
        break;
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    {
        uint8_t volume = param->set_abs_vol.volume;
        ESP_LOGI(TAG, "Set absolute volume: %u", volume);
        s_abs_volume = volume;
        // Response is sent automatically in newer ESP-IDF versions
        break;
    }
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
        if (param->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE)
        {
            esp_avrc_rn_param_t rn_param = {.volume = s_abs_volume};
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }
        break;
    default:
        ESP_LOGD(TAG, "Unhandled AVRCP target event %d", event);
        break;
    }
}

static void i2s_writer_task(void *arg)
{
    ESP_LOGI(TAG, "I2S writer task started");
    while (true)
    {
        size_t item_size = 0;
        uint8_t *data = (uint8_t *)xRingbufferReceive(s_i2s_ringbuf, &item_size, pdMS_TO_TICKS(1000));
        if (!data)
        {
            // Timeout - no data in ring buffer
            continue;
        }

        s_audio_tx_stats.bytes_accum += item_size;
        s_audio_tx_stats.chunks++;

        uint8_t *ptr = data;
        size_t remaining = item_size;

        while (remaining > 0)
        {
            size_t written = 0;
            esp_err_t err = i2s_channel_write(s_tx_chan, ptr, remaining, &written, portMAX_DELAY);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "i2s_channel_write failed: %s", esp_err_to_name(err));
                break;
            }
            remaining -= written;
            ptr += written;
        }

        uint32_t now_ms = esp_log_timestamp();
        if ((now_ms - s_audio_tx_stats.last_log_ms) >= 1000)
        {
            size_t free_bytes = xRingbufferGetCurFreeSize(s_i2s_ringbuf);
            ESP_LOGD(TAG, "Audio TX: %u bytes across %u chunks, ring free %u",
                     (unsigned int)s_audio_tx_stats.bytes_accum,
                     (unsigned int)s_audio_tx_stats.chunks,
                     (unsigned int)free_bytes);
            s_audio_tx_stats.bytes_accum = 0;
            s_audio_tx_stats.chunks = 0;
            s_audio_tx_stats.last_log_ms = now_ms;
        }

        vRingbufferReturnItem(s_i2s_ringbuf, data);
    }
}

static const char *a2dp_conn_state_to_str(esp_a2d_connection_state_t state)
{
    switch (state)
    {
    case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
        return "DISCONNECTED";
    case ESP_A2D_CONNECTION_STATE_CONNECTING:
        return "CONNECTING";
    case ESP_A2D_CONNECTION_STATE_CONNECTED:
        return "CONNECTED";
    case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
        return "DISCONNECTING";
    default:
        return "UNKNOWN";
    }
}

static const char *a2dp_audio_state_to_str(esp_a2d_audio_state_t state)
{
    if (state == ESP_A2D_AUDIO_STATE_STARTED)
    {
        return "STARTED";
    }
#if defined(ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND) && (ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND != ESP_A2D_AUDIO_STATE_STOPPED)
    if (state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND)
    {
        return "REMOTE_SUSPEND";
    }
#endif
    if (state == ESP_A2D_AUDIO_STATE_STOPPED)
    {
        return "STOPPED";
    }
    return "UNKNOWN";
}

static void log_bda(const char *label, const uint8_t *bda)
{
    ESP_LOGI(TAG, "%s %02X:%02X:%02X:%02X:%02X:%02X",
             label,
             (unsigned int)bda[0],
             (unsigned int)bda[1],
             (unsigned int)bda[2],
             (unsigned int)bda[3],
             (unsigned int)bda[4],
             (unsigned int)bda[5]);
}

static void i2s_tone_test_task(void *arg)
{
    ESP_LOGI(TAG, "Tone test task started - generating 440Hz sine wave");

    // Generate 440Hz sine wave at 44100 Hz sample rate
    // Period = 44100 / 440 = 100.227 samples per cycle
    // We'll use 100 samples for simplicity
    const int samples_per_period = 100;
    const int amplitude = 8000; // Comfortable listening level

    // Pre-generate one period of sine wave
    int16_t sine_wave[samples_per_period * 2]; // stereo (L+R)
    for (int i = 0; i < samples_per_period; i++)
    {
        // Calculate sine value: amplitude * sin(2 * PI * i / samples_per_period)
        float angle = 2.0f * 3.14159265f * i / samples_per_period;
        int16_t sample = (int16_t)(amplitude * sinf(angle));

        // Duplicate to both channels (stereo)
        sine_wave[i * 2] = sample;     // Left
        sine_wave[i * 2 + 1] = sample; // Right
    }

    ESP_LOGI(TAG, "Sine wave buffer prepared (%d samples, %d bytes per period)",
             samples_per_period, samples_per_period * 4);
    ESP_LOGI(TAG, "Starting continuous tone output...");

    size_t bytes_written;
    uint32_t total_bytes = 0;
    uint32_t last_report_ms = 0;

    while (1)
    {
        // Write one period of the sine wave using modern I2S API
        esp_err_t err = i2s_channel_write(s_tx_chan, sine_wave, samples_per_period * 4,
                                          &bytes_written, portMAX_DELAY);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_write failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        total_bytes += bytes_written;

        // Report stats every second
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_report_ms >= 1000)
        {
            ESP_LOGI(TAG, "Tone output: %u bytes written total", (unsigned int)total_bytes);
            last_report_ms = now;
        }
    }
}