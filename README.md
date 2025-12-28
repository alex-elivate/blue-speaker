# Blue Speaker - ESP32 Bluetooth A2DP Speaker

A Bluetooth audio receiver for ESP32 that streams A2DP audio to an I2S amplifier (MAX98357 or similar).

## Hardware

- **MCU:** ESP32 WROOM (ESP32-D0WD)
- **Amplifier:** MAX98357 I2S Audio Amplifier
- **Flash:** 4MB+
- **Features:** Bluetooth Classic A2DP + AVRCP

## Pin Configuration

Connect your MAX98357 I2S amplifier to the ESP32:

| Signal | ESP32 GPIO | MAX98357 Pin  |
| ------ | ---------- | ------------- |
| BCLK   | GPIO 27    | BCLK          |
| LRCK   | GPIO 26    | LRC           |
| DOUT   | GPIO 25    | DIN           |
| SD     | GPIO 33    | SD (Shutdown) |
| GND    | GND        | GND           |
| Power  | 5V         | VIN           |

> **Note:** GPIO 33 controls the MAX98357 shutdown pin (HIGH = enabled, LOW = disabled).

## Software Configuration

- **Framework:** ESP-IDF 5.5.1
- **Platform:** PlatformIO
- **Bluetooth Mode:** Classic (BR/EDR) only - BLE disabled to save memory
- **Audio Codec:** SBC (A2DP)
- **Sample Rate:** 44.1 kHz (default, fixed)
- **I2S Mode:** Philips I2S standard, 16-bit stereo
- **I2S Driver:** Modern `driver/i2s_std.h` API

## Features

- Bluetooth A2DP audio sink (receive audio from phone/computer)
- AVRCP transport control (play/pause/volume)
- Automatic pairing (PIN: 0000)
- Discoverable as "Blue Speaker"
- Ring buffer for smooth audio playback
- Dynamic sample rate support (16/32/44.1/48 kHz)

## Building and Flashing

```bash
# Build the project
pio run

# Upload to ESP32
pio run -t upload

# Monitor serial output
pio device monitor
```

## Usage

1. **Power on** the ESP32
2. **Search for Bluetooth devices** on your phone/computer
3. **Connect to "Blue Speaker"**
4. **Play audio** - it will stream to the I2S amplifier

### Pairing

- Device name: **Blue Speaker**
- PIN code: **0000** (if prompted)
- Auto-accepts pairing requests

## Configuration

### Bluetooth Settings

Configured in `sdkconfig.defaults`:

- Bluetooth Classic enabled
- BR/EDR mode only (BLE disabled for lower memory usage)
- A2DP sink profile enabled
- AVRCP target + controller enabled

### Audio Settings

[src/main.c](src/main.c):

- Default sample rate: 44,100 Hz (fixed)
- Bits per sample: 16-bit
- Channel format: Stereo (L/R)
- DMA buffer: 8 buffers × 64 bytes
- Ring buffer: 32 KB
- I2S format: Philips standard (correct for MAX98357)

### Test Tone Mode

For hardware debugging, set `TEST_TONE_MODE` to `1` in [src/main.c](src/main.c#L29) to generate a 440Hz sine wave instead of Bluetooth audio.

### Customization

To change the device name, edit [src/main.c](src/main.c#L31):

```c
#define BT_SPEAKER_NAME "Blue Speaker"
```

To change I2S pins, edit [src/main.c](src/main.c#L37-L40):

```c
#deArchitecture

### Audio Pipeline

```

Bluetooth Source (Phone/Computer)
↓ A2DP SBC codec
bt_a2d_data_cb() - Receives decoded PCM audio
↓ Ring buffer (32KB)
i2s_writer_task() - FreeRTOS task
↓ Modern I2S driver
MAX98357 I2S Amplifier
↓
Speaker

```

### Key Components

1. **Bluetooth Stack**: Classic Bluetooth (BR/EDR) with A2DP sink and AVRCP profiles
2. **Ring Buffer**: Decouples Bluetooth callbacks from I2S writes to prevent underruns
3. **I2S Writer Task**: Dedicated FreeRTOS task for smooth audio output
4. **Statistics**: RX/TX byte tracking with periodic logging

## Project History

### ESP-IDF 5.5.1 Migration

This project was updated for ESP-IDF 5.5.1 with the following changes:

1. **Modern I2S Driver**
   - Migrated from legacy driver to `driver/i2s_std.h` API
   - Uses `i2s_channel_*` functions with proper config structs
   - Philips I2S standard format for MAX98357 compatibility

2. **API Compatibility Updates**
   - Updated `esp_bt_dev_set_device_name()` → `esp_bt_gap_set_device_name()`
   - Removed deprecated `device_name` field from GAP callback
Approximate footprint (ESP32-S3):
```

RAM: ~50-60KB (includes Bluetooth stack and ring buffer)
Flash: ~800KB (Bluetooth classic + A2DP/AVRCP profil

- BR/EDR mode only (BLE disabled via `esp_bt_controller_mem_release()`)
- Proper A2DP sink and AVRCP target/controller initialization
- PIN code "0000" with SSP auto-confirm

## Verified Working

- Compiles successfully
- Check serial monitor for "Bluetooth stack initialized" message
- Try power cycling the ESP32

### No audio output

1. **Test I2S hardware**: Set `TEST_TONE_MODE` to `1` and rebuild - you should hear a 440Hz tone
2. **Check wiring**: Verify BCLK, LRCK, DOUT connections
3. **SD pin**: Ensure GPIO 33 is HIGH (amplifier enabled)
4. **Power**: MAX98357 needs 5V (3.3V may work but lower volume)
5. **Speaker**: Check speaker/headphone connection to amplifier output

### Pairing fails

- Default PIN is 0000
- Device auto-accepts most pairing requests
- Try forgetting device and re-pairing
- Check serial logs for authentication errors

### Audio stuttering/dropouts

- Ring buffer may be too small - increase `I2S_RINGBUFFER_SIZE_BYTES`
- Reduce logging verbosity (remove ESP_LOGI calls from hot paths)
- Check Wi-Fi interference (if enabled)

## Known Issues

1. **Sample rate fixed at 44.1kHz**: Dynamic sample rate detection from A2DP codec info is not implemented
2. **Volume control**: AVRCP volume commands are received but not applied to I2S output
3. **Verbose logging**: I2S writer task logs every chunk in debug mode - may impact performance

## Future Improvements

- [ ] Implement dynamic sample rate detection from A2DP codec
- [ ] Apply AVRCP volume control to I2S output (software volume scaling)
- [ ] Reduce logging in hot paths for better performance
- [ ] Add support for multiple sample rates (16/32/48 kHz)
- [ ] Implement battery monitoring (if running on battery)

## License

This project is provided as-is for educational and development purposes.
Flash: 19.4% (811,701 / 4,194,304 bytes)
IRAM: 84.94% (111,335 / 131,072 bytes)

```

## Troubleshooting

### Can't find "Blue Speaker"
- Ensure Bluetooth is enabled on your device
- Device should be discoverable after boot
- Try power cycling the ESP32

### No audio output
- Check I2S wiring
- Verify MAX98357 has power
- Check speaker/headphone connection to amplifier

### Pairing fails
- Default PIN is 0000
- Device auto-accepts most pairing requests
- Try forgetting device and re-pairing

## License

This project is provided as-is for educational and development purposes.

## Notes

- Legacy I2S driver used (works but consider migrating to `driver/i2s_std.h` in future)
- Sample rate currently fixed at 44.1kHz (dynamic rate detection simplified for compatibility)
- Volume control receives commands but doesn't apply to I2S output (can be added)
```
