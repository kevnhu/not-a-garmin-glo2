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
cd "/Users/kevin/Desktop/EFB Bluetooth GPS"
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
(Replace `/dev/cu.usbserial-*` with your actual port. Use `ls /dev/cu.*` to find it on macOS)

Monitor serial output:
```bash
idf.py -p /dev/cu.usbserial-* monitor
```

Or build + flash + monitor in one command:
```bash
idf.py -p /dev/cu.usbserial-* flash monitor
```Build Errors
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

### OLED Display Not Working
- Check I2C address (default is 0x3C, some displays use 0x3D)
- Modify `OLED_ADDRESS` in main/main.c if needed
- Verify wiring (SDA/SCL connections)
- Check power to OLED
- OLED display currently shows status in serial monitor (full graphics implementation can be added)
  - Altitude
  - Ground speed
  - Number of satellites

## Troubleshooting

### No GPS Fix
- Ensure you're outdoors or near a window
- GPS needs clear view of sky
- First fix can take 1-2 minutes (cold start)
- Check wiring connections
- Verify GPS module has power (LED should blink)

### Bluetooth Won't Connect
- Make sure ESP32 is powered on
- Check Serial Monitor (115200 baud) for "Bluetooth initialized" message
- Try forgetting device in Bluetooth settings and re-pairing
- Restart ESP32

### No Data in ForeFlight
- Check OLED display shows satellites and fix
- Verify Bluetooth is connected
- In ForeFlight Devices, check if GPS is receiving data
- Try disconnecting and reconnecting Bluetooth

### OLED Display Not Working
- Check I2C address (default is 0x3C, some displays use 0x3D)
- Verify wiring (SDA/SCL connections)
- Check power to OLED

## Power Options

### USB Power
- Connect ESP32 to USB power bank or USB charger

### Battery Power
- Use 3.7V LiPo battery with ESP32 battery connector
- Or use 5V power bank connected to ESP32 USB port

### Estimated Battery Life
- USB power bank (10,000mAh): ~24+ hours
You can configure the project using menuconfig:
```bash
idf.py menuconfig
```

Navigate to: `ForeFlight GPS Configuration`

### Change Bluetooth Device Name
Edit in main/main.c:
```c
#define BT_DEVICE_NAME "ForeFlight GPS"
```

Or use menuconfig: `Component config → Bluetooth → Bluetooth controller`

### Adjust GPS Baud Rate
Edit in main/main.c:
```c
#define GPS_BAUD_RATE 9600
```
Framework**: ESP-IDF (Pure C with FreeRTOS)
- **Bluetooth Profile**: SPP (Serial Port Profile) - Classic Bluetooth
- **GPS Output Format**: NMEA 0183
- **Update Rate**: 1Hz (default M8N setting)
- **Accuracy**: ~2.5m CEP (M8N specification)
- **RTOS Tasks**: 
  - GPS task (priority 5): Reads UART and forwards to Bluetooth
  - Display task (priority 4): Updates OLED every 2 seconds
- **Memory**: Classic BT only (BLE disabled to save memory)

## Project Structure

```
EFB Bluetooth GPS/
├── CMakeLists.txt              # Root build configuration
├── main/
│   ├── CMakeLists.txt          # Component build config
│   ├── Kconfig.projbuild       # Configuration menu
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

### Partition Table
Default partition table is used. To customize, add `partitions.csv`.
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

### Enable Full OLED Graphics
The current implementation logs to serial. To add full graphics:
1. Integrate a font library (u8g2, etc.)
2. Implement text rendering in oled_draw_text()
3. Add graphics primitives as needed Adjust GPS Baud Rate
If your M8N uses different baud rate, change:
```cpp
#define GPS_BAUD 9600
```

### Change Display Update Rate
Modify this value (in milliseconds):
```cpp
if (millis() - lastUpdateTime > 2000) {  // 2000 = 2 seconds
```

## Technical Notes

- **Bluetooth Profile**: SPP (Serial Port Profile)
- **GPS Output Format**: NMEA 0183
- **Update Rate**: 1Hz (default M8N setting)
- **Accuracy**: ~2.5m CEP (M8N specification)

## License

Free to use and modify for personal projects.
