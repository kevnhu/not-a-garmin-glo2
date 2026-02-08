# ForeFlight WiFi GPS (ESP-IDF)

> This project was entirely vibe coded with [Claude](https://claude.ai) by Anthropic.

A WiFi GPS receiver for ForeFlight using ESP32, M8N GPS module, and OLED display. Built with ESP-IDF framework in pure C with FreeRTOS.

The ESP32 creates a WiFi access point and sends GPS data to ForeFlight using the GDL90 protocol over UDP. Just connect your iPad to the "ForeFlight GPS" WiFi network and ForeFlight automatically picks up the GPS data - no pairing or certification needed.

## Hardware Requirements

- ESP32 development board
- M8N GPS module (u-blox, 115200 baud, UBX protocol)
- 128x64 OLED display (I2C, SSD1306)
- Jumper wires
- USB cable for programming
- Battery/power source (optional, for portable use)

## Wiring Connections

### M8N GPS Module
- **GPS TX** → ESP32 **GPIO16** (RX2)
- **GPS RX** → ESP32 **GPIO17** (TX2)
- **VCC** → ESP32 **5V** pin (passes USB voltage through)
- **GND** → **GND**

### OLED Display (I2C)
- **SDA** → ESP32 **GPIO21**
- **SCL** (also labeled SCK on some modules) → ESP32 **GPIO22**
- **VCC** → ESP32 **3.3V** pin
- **GND** → **GND**

### Power Notes
- The ESP32 3.3V pin supplies 3.3V when powered via USB
- The ESP32 5V/VIN pin passes through USB voltage (~5V)
- The M8N GPS module can run on 3.3V or 5V (check your module)
- The OLED display runs on 3.3V

## Software Setup

### Prerequisites
- **Python 3.10+** (ESP-IDF v6.x requirement)
  - macOS: `brew install python@3.12`
- **cmake**: `brew install cmake`

### 1. Install ESP-IDF
Follow the official ESP-IDF installation guide:
- **macOS/Linux**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html
- **Windows**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html

Quick install (macOS/Linux):
```bash
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
. ./export.sh
```

### 2. Build and Flash

Set up environment (run this in every new terminal):
```bash
. ~/esp/esp-idf/export.sh
```

Navigate to your project directory:
```bash
cd "/Users/kevin/Desktop/EFB Bluetooth GPS"
```

Build the project:
```bash
idf.py build
```

Flash to ESP32:
```bash
idf.py -p /dev/cu.usbserial-* flash
```
(Replace `/dev/cu.usbserial-*` with your actual port. Use `ls /dev/cu.*` to find it on macOS)

Monitor serial output:
```bash
idf.py -p /dev/cu.usbserial-* monitor
```

Or build + flash + monitor in one command:
```bash
idf.py -p /dev/cu.usbserial-* flash monitor
```

## OLED Display

The OLED display shows real-time GPS and Bluetooth status using a built-in 5x7 pixel font:
- **Line 1**: "ForeFlight GPS" title
- **Line 2**: Bluetooth connection status (Connected / Waiting...)
- **Line 3**: GPS fix status (FIX / Searching)
- **Line 4**: Number of satellites
- **Line 5**: Altitude (meters)
- **Line 6**: Latitude
- **Line 7**: Longitude

## How It Works

1. **GPS**: The M8N module communicates at 115200 baud using UBX protocol natively. At startup, the firmware sends a UBX CFG-PRT command to switch the module's output to NMEA 0183 format, which ForeFlight understands.
2. **BLE**: The ESP32 advertises as "ForeFlight GPS" using the Nordic UART Service (NUS). When ForeFlight connects, it subscribes to notifications on the TX characteristic. The firmware forwards raw NMEA sentences from the GPS to the BLE TX characteristic in 20-byte chunks.
3. **Display**: A separate FreeRTOS task updates the OLED every 2 seconds with parsed GPS data.

## Troubleshooting

### Build Errors
- Make sure ESP-IDF environment is sourced: `. ~/esp/esp-idf/export.sh`
- Check ESP-IDF version: `idf.py --version` (v6.1+ tested)
- Ensure Python 3.10+ is installed: `python3 --version`
- Ensure cmake is installed: `cmake --version`
- Clean build: `idf.py fullclean` then `idf.py build`

### No GPS Fix
- Ensure you're outdoors or near a window
- GPS needs clear view of sky
- First fix can take 1-2 minutes (cold start)
- Check wiring connections
- Verify GPS module has power (LED should blink)
- Monitor serial output: `idf.py monitor` to see NMEA sentences

### Bluetooth Won't Connect
- Make sure ESP32 is powered on
- Check serial monitor for "BLE advertising started" message
- The device advertises as "ForeFlight GPS" via BLE
- iOS requires BLE (not Classic Bluetooth) - this project uses BLE NUS
- Try forgetting device in Bluetooth settings and re-pairing
- Restart ESP32

### No Data in ForeFlight
- Check OLED display shows satellites and fix
- Verify Bluetooth is connected (serial log: "BLE client connected")
- Verify notifications are enabled (serial log: "BLE notifications enabled")
- In ForeFlight Devices, check if GPS is receiving data
- Try disconnecting and reconnecting Bluetooth

### OLED Display Not Working
- The firmware scans the I2C bus at startup and logs found addresses
- Check serial monitor for "I2C device found at address 0x3C"
- If your display uses 0x3D, modify `OLED_ADDRESS` in main/main.c
- Verify wiring: SDA → GPIO21, SCL → GPIO22
- Check power to OLED (3.3V)

## Power Options

### USB Power
- Connect ESP32 to USB power bank or USB charger

### Battery Power
- Use 3.7V LiPo battery with ESP32 battery connector
- Or use 5V power bank connected to ESP32 USB port

### Estimated Battery Life
- USB power bank (10,000mAh): ~24+ hours

## Configuration

### Change Bluetooth Device Name
Edit in main/main.c:
```c
#define BLE_DEVICE_NAME "ForeFlight GPS"
```

### Adjust GPS Baud Rate
Edit in main/main.c (must match your GPS module):
```c
#define GPS_BAUD_RATE 115200
```

### Change Pin Assignments
Edit in main/main.c:
```c
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
```

### Change Display Update Rate
Modify in display_task() function (milliseconds):
```c
vTaskDelay(pdMS_TO_TICKS(2000));  // 2000 = 2 seconds
```

## Technical Specifications

- **Framework**: ESP-IDF v6.1 (Pure C with FreeRTOS)
- **Bluetooth**: BLE with Nordic UART Service (NUS)
  - Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
  - TX (notify): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
  - RX (write): 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
- **GPS Module**: u-blox M8N at 115200 baud
- **GPS Output Format**: NMEA 0183 (converted from UBX at startup)
- **Update Rate**: 10Hz (M8N default)
- **Accuracy**: ~2.5m CEP (M8N specification)
- **OLED**: SSD1306 128x64, I2C at 0x3C, built-in 5x7 font
- **RTOS Tasks**:
  - GPS task (priority 5): Reads UART, parses NMEA, forwards to BLE
  - Display task (priority 4): Updates OLED every 2 seconds
- **Memory**: BLE only (Classic BT disabled to save memory)

## Project Structure

```
EFB Bluetooth GPS/
├── CMakeLists.txt              # Root build configuration
├── main/
│   ├── CMakeLists.txt          # Component build config
│   ├── Kconfig.projbuild       # Configuration menu
│   └── main.c                  # Main application (C code)
├── sdkconfig                   # ESP-IDF build configuration
└── README.md                   # This file
```

## Development Tips

### View Serial Output
```bash
idf.py -p /dev/cu.usbserial-* monitor
```

### Clean Build
```bash
idf.py fullclean
idf.py build
```

### Erase Flash Completely
```bash
idf.py erase-flash
idf.py flash
```

### Check Code Size
```bash
idf.py size
```

## License

Free to use and modify for personal projects.
