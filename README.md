# ForeFlight Bluetooth GPS (ESP-IDF)

A Bluetooth GPS receiver for ForeFlight using ESP32, M8N GPS module, and OLED display. Built with ESP-IDF framework in pure C with FreeRTOS.

## Hardware Requirements

- ESP32 development board
- M8N GPS module
- 128x64 OLED display (I2C, SSD1306)
- Jumper wires
- USB cable for programming
- Battery/power source (optional, for portable use)

## Wiring Connections

### M8N GPS Module
- **GPS TX** → ESP32 **GPIO16** (RX2)
- **GPS RX** → ESP32 **GPIO17** (TX2)
- **VCC** → **3.3V** or **5V** (check your module)
- **GND** → **GND**

### OLED Display (I2C)
- **SDA** → ESP32 **GPIO21**
- **SCL** → ESP32 **GPIO22**
- **VCC** → **3.3V**
- **GND** → **GND**

## Software Setup

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

Navigate to your project directory:
```bash
cd Foreflight-Bluetooth-GPS
```

Set up environment (run this in every new terminal):
```bash
. ~/esp/esp-idf/export.sh
```

Configure the project (optional):
```bash
idf.py menuconfig
```

Build the project:
```bash
idf.py build
```

Flash to ESP32:
```bash
idf.py -p /dev/cu.usbserial-* flash
```
(Replace `/dev/cu.usbserial-*` with your actual port. Use `ls /dev/cu.*` to find it on macOS, or `ls /dev/ttyUSB*` on Linux)

Monitor serial output:
```bash
idf.py -p /dev/cu.usbserial-* monitor
```

Or build + flash + monitor in one command:
```bash
idf.py -p /dev/cu.usbserial-* flash monitor
```

## Troubleshooting

### Build Errors
- Make sure ESP-IDF environment is sourced: `. ~/esp/esp-idf/export.sh`
- Check ESP-IDF version: `idf.py --version` (v4.4+ recommended)
- Clean build: `idf.py fullclean` then `idf.py build`
- Check Python version: ESP-IDF requires Python 3.7+

### No GPS Fix
- Ensure you're outdoors or near a window
- GPS needs clear view of sky
- First fix can take 1-2 minutes (cold start)
- Check wiring connections
- Verify GPS module has power (LED should blink)
- Monitor serial output: `idf.py monitor` to see GPS data

### Bluetooth Won't Connect
- Make sure ESP32 is powered on
- Check serial monitor for "SPP initialized" message
- Try forgetting device in Bluetooth settings and re-pairing
- Restart ESP32
- Check Bluetooth Classic is enabled (BLE is disabled in this project)

### No Data in ForeFlight
- Monitor serial output to verify GPS sentences are being received
- Verify Bluetooth is connected (check serial logs)
- In ForeFlight Devices, check if GPS is receiving data
- Try disconnecting and reconnecting Bluetooth
- Ensure GPS has a valid fix (satellites visible)

### OLED Display Not Working
- Check I2C address (default is 0x3C, some displays use 0x3D)
- Modify `OLED_ADDRESS` in main/main.c if needed
- Verify wiring (SDA/SCL connections)
- Check power to OLED
- Note: OLED display currently shows status in serial monitor (full graphics implementation can be added with a font library)

## Power Options

### USB Power
- Connect ESP32 to USB power bank or USB charger

### Battery Power
- Use 3.7V LiPo battery with ESP32 battery connector
- Or use 5V power bank connected to ESP32 USB port

### Estimated Battery Life
- USB power bank (10,000mAh): ~24+ hours

## Configuration

You can configure the project using menuconfig:
```bash
idf.py menuconfig
```

Navigate to: `ForeFlight GPS Configuration`

Available options:
- GPS UART port number (default: 2)
- GPS RX GPIO pin (default: 16)
- GPS TX GPIO pin (default: 17)
- GPS baud rate (default: 9600)
- Bluetooth device name (default: "ForeFlight GPS")

### Change Bluetooth Device Name
Via menuconfig: `ForeFlight GPS Configuration → Bluetooth device name`

Or edit in main/main.c (line 64):
```c
#define CONFIG_BT_DEVICE_NAME "Your Custom Name"
```

### Adjust GPS Pins
Via menuconfig: `ForeFlight GPS Configuration → GPS RX/TX GPIO`

Or edit in main/main.c:
```c
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
```

### Change I2C Pins for OLED
Edit in main/main.c:
```c
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
```

### Change Display Update Rate
Modify in display_task() function (milliseconds):
```c
vTaskDelay(pdMS_TO_TICKS(2000));  // 2000 = 2 seconds
```

### Enable Full OLED Graphics
The current implementation logs to serial. To add full graphics:
1. Integrate a font library (u8g2, Adafruit GFX, etc.)
2. Implement text rendering in oled_draw_text()
3. Add graphics primitives as needed

## Technical Specifications

- **Framework**: ESP-IDF (Pure C with FreeRTOS)
- **Bluetooth Profile**: SPP (Serial Port Profile) - Classic Bluetooth
- **GPS Output Format**: NMEA 0183 (GGA and RMC sentences)
- **Update Rate**: 1Hz (default M8N setting)
- **Accuracy**: ~2.5m CEP (M8N specification)
- **RTOS Tasks**:
  - GPS task (priority 5): Reads UART and forwards to Bluetooth
  - Display task (priority 4): Updates OLED every 2 seconds
- **Memory**: Classic BT only (BLE disabled to save memory)
- **Display Info**:
  - Bluetooth connection status
  - GPS fix status and satellite count
  - Current latitude/longitude
  - Altitude

## Project Structure

```
Foreflight-Bluetooth-GPS/
├── CMakeLists.txt              # Root build configuration
├── sdkconfig.defaults          # ESP-IDF default configuration
├── main/
│   ├── CMakeLists.txt          # Component build config
│   ├── Kconfig.projbuild       # Configuration menu definitions
│   └── main.c                  # Main application (C code)
├── README.md                   # This file
└── EFB_Bluetooth_GPS.ino       # Old Arduino version (deprecated)
```

## Development Tips

### View Serial Output
```bash
idf.py monitor
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

### Check Partition Usage
```bash
idf.py partition-table
```

### Exit Monitor
Press `Ctrl+]` to exit the serial monitor

## License

Free to use and modify for personal projects.
