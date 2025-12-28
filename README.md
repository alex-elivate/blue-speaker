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
| SD     | GPIO 33    | SD            |
| GND    | GND        | GND           |
| Power  | 5V         | VIN           |


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

### Pairing

- Device name: **Blue Speaker**
- PIN code: **0000** (if prompted)
- Auto-accepts pairing requests


### Test Tone Mode

For hardware debugging, set `TEST_TONE_MODE` to `1` in [src/main.c](src/main.c#L29) to generate a 440Hz sine wave instead of Bluetooth audio.

### Customization

To change the device name, edit [src/main.c](src/main.c#L31):

```c
#define BT_SPEAKER_NAME "Blue Speaker"
```

## License

This project is provided as-is for educational and development purposes.

