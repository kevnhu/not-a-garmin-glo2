/*
 * ForeFlight Bluetooth GPS
 * ESP32 + M8N GPS + OLED Display
 * 
 * Connections:
 * M8N GPS:
 *   - GPS TX -> ESP32 GPIO16 (RX2)
 *   - GPS RX -> ESP32 GPIO17 (TX2)
 *   - VCC -> 3.3V or 5V
 *   - GND -> GND
 * 
 * OLED Display (I2C):
 *   - SDA -> GPIO21
 *   - SCL -> GPIO22
 *   - VCC -> 3.3V
 *   - GND -> GND
 */

#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// GPS Serial (using UART2)
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// Bluetooth
BluetoothSerial SerialBT;
String deviceName = "ForeFlight GPS";

// GPS Data
String nmeaSentence = "";
int satellites = 0;
bool gpsFixed = false;
String latitude = "";
String longitude = "";
String altitude = "";
unsigned long lastUpdateTime = 0;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("ForeFlight Bluetooth GPS Starting...");
  
  // Initialize GPS Serial
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS Serial initialized");
  
  // Initialize Bluetooth
  if (!SerialBT.begin(deviceName)) {
    Serial.println("Bluetooth init failed!");
  } else {
    Serial.println("Bluetooth initialized: " + deviceName);
  }
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED init failed!");
  } else {
    Serial.println("OLED initialized");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ForeFlight GPS");
    display.println("Waiting for GPS...");
    display.display();
  }
  
  delay(1000);
}

void loop() {
  // Read GPS data
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    
    // Forward to Bluetooth
    if (SerialBT.hasClient()) {
      SerialBT.write(c);
    }
    
    // Build NMEA sentence for parsing
    if (c == '$') {
      nmeaSentence = "$";
    } else if (c == '\n') {
      nmeaSentence += c;
      parseNMEA(nmeaSentence);
      nmeaSentence = "";
    } else {
      nmeaSentence += c;
    }
  }
  
  // Update display every 2 seconds
  if (millis() - lastUpdateTime > 2000) {
    updateDisplay();
    lastUpdateTime = millis();
  }
}

void parseNMEA(String sentence) {
  // Parse GGA sentence for fix and satellite info
  if (sentence.startsWith("$GPGGA") || sentence.startsWith("$GNGGA")) {
    int commaCount = 0;
    int startIdx = 0;
    
    for (int i = 0; i < sentence.length(); i++) {
      if (sentence[i] == ',') {
        commaCount++;
        
        // Quality indicator (6th field)
        if (commaCount == 6) {
          int quality = sentence.substring(i + 1, sentence.indexOf(',', i + 1)).toInt();
          gpsFixed = (quality > 0);
        }
        
        // Number of satellites (7th field)
        if (commaCount == 7) {
          satellites = sentence.substring(i + 1, sentence.indexOf(',', i + 1)).toInt();
        }
        
        // Altitude (9th field)
        if (commaCount == 9) {
          altitude = sentence.substring(i + 1, sentence.indexOf(',', i + 1));
        }
      }
    }
  }
  
  // Parse RMC sentence for coordinates
  if (sentence.startsWith("$GPRMC") || sentence.startsWith("$GNRMC")) {
    int commaCount = 0;
    
    for (int i = 0; i < sentence.length(); i++) {
      if (sentence[i] == ',') {
        commaCount++;
        
        // Latitude (3rd field)
        if (commaCount == 3) {
          int nextComma = sentence.indexOf(',', i + 1);
          latitude = sentence.substring(i + 1, nextComma);
          // Get N/S indicator
          i = nextComma;
          nextComma = sentence.indexOf(',', i + 1);
          latitude += sentence.substring(i + 1, nextComma);
        }
        
        // Longitude (5th field)
        if (commaCount == 5) {
          int nextComma = sentence.indexOf(',', i + 1);
          longitude = sentence.substring(i + 1, nextComma);
          // Get E/W indicator
          i = nextComma;
          nextComma = sentence.indexOf(',', i + 1);
          longitude += sentence.substring(i + 1, nextComma);
        }
      }
    }
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  // Title
  display.setTextSize(1);
  display.println("ForeFlight GPS");
  display.println("---------------");
  
  // Bluetooth status
  if (SerialBT.hasClient()) {
    display.println("BT: Connected");
  } else {
    display.println("BT: Waiting...");
  }
  
  // GPS Status
  if (gpsFixed) {
    display.print("GPS: FIX (");
    display.print(satellites);
    display.println(" sats)");
  } else {
    display.print("GPS: Searching (");
    display.print(satellites);
    display.println(")");
  }
  
  // Position info
  if (latitude.length() > 0) {
    display.print("Lat: ");
    display.println(formatCoordinate(latitude));
  }
  
  if (longitude.length() > 0) {
    display.print("Lon: ");
    display.println(formatCoordinate(longitude));
  }
  
  if (altitude.length() > 0 && gpsFixed) {
    display.print("Alt: ");
    display.print(altitude);
    display.println("m");
  }
  
  display.display();
}

String formatCoordinate(String coord) {
  // Simple formatting - just truncate if too long
  if (coord.length() > 11) {
    return coord.substring(0, 11);
  }
  return coord;
}
