/*
 * ForeFlight Bluetooth GPS - ESP-IDF Version
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "nvs_flash.h"

// Configuration - use Kconfig values where available
#ifndef CONFIG_GPS_UART_NUM
#define CONFIG_GPS_UART_NUM 2
#endif
#ifndef CONFIG_GPS_RX_PIN
#define CONFIG_GPS_RX_PIN 16
#endif
#ifndef CONFIG_GPS_TX_PIN
#define CONFIG_GPS_TX_PIN 17
#endif
#ifndef CONFIG_GPS_BAUD_RATE
#define CONFIG_GPS_BAUD_RATE 9600
#endif
#ifndef CONFIG_BT_DEVICE_NAME
#define CONFIG_BT_DEVICE_NAME "ForeFlight GPS"
#endif

#define GPS_UART_NUM        (UART_NUM_0 + CONFIG_GPS_UART_NUM)
#define GPS_RX_PIN          CONFIG_GPS_RX_PIN
#define GPS_TX_PIN          CONFIG_GPS_TX_PIN
#define GPS_BAUD_RATE       CONFIG_GPS_BAUD_RATE
#define GPS_BUF_SIZE        1024

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define OLED_ADDRESS        0x3C

#define BT_DEVICE_NAME      CONFIG_BT_DEVICE_NAME
#define SPP_SERVER_NAME     "SPP_SERVER"

// OLED Commands
#define OLED_CMD_DISPLAY_OFF    0xAE
#define OLED_CMD_DISPLAY_ON     0xAF
#define OLED_CMD_SET_CONTRAST   0x81
#define OLED_CMD_DISPLAY_NORMAL 0xA6
#define OLED_CMD_SET_MUX_RATIO  0xA8
#define OLED_CMD_SET_DISPLAY_OFFSET 0xD3
#define OLED_CMD_SET_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP 0xA1
#define OLED_CMD_SET_COM_SCAN_DEC 0xC8
#define OLED_CMD_SET_COM_PINS   0xDA
#define OLED_CMD_SET_PRECHARGE  0xD9
#define OLED_CMD_SET_VCOM_DETECT 0xDB
#define OLED_CMD_CHARGE_PUMP    0x8D
#define OLED_CMD_DEACTIVATE_SCROLL 0x2E

static const char *TAG = "GPS";

// GPS data structure
typedef struct {
    int satellites;
    bool gps_fixed;
    char latitude[16];
    char longitude[16];
    char altitude[16];
} gps_data_t;

static gps_data_t gps_data = {0};
static uint32_t spp_handle = 0;
static bool bt_connected = false;

// I2C Functions
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t oled_write_cmd(uint8_t cmd)
{
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, 0x00, true); // Command mode
    i2c_master_write_byte(i2c_cmd, cmd, true);
    i2c_master_stop(i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(i2c_cmd);
    return ret;
}

static esp_err_t oled_write_data(uint8_t *data, size_t len)
{
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, 0x40, true); // Data mode
    i2c_master_write(i2c_cmd, data, len, true);
    i2c_master_stop(i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(i2c_cmd);
    return ret;
}

static void oled_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    
    oled_write_cmd(OLED_CMD_DISPLAY_OFF);
    oled_write_cmd(OLED_CMD_SET_MUX_RATIO);
    oled_write_cmd(0x3F);
    oled_write_cmd(OLED_CMD_SET_DISPLAY_OFFSET);
    oled_write_cmd(0x00);
    oled_write_cmd(OLED_CMD_SET_START_LINE | 0x00);
    oled_write_cmd(OLED_CMD_SET_SEGMENT_REMAP);
    oled_write_cmd(OLED_CMD_SET_COM_SCAN_DEC);
    oled_write_cmd(OLED_CMD_SET_COM_PINS);
    oled_write_cmd(0x12);
    oled_write_cmd(OLED_CMD_SET_CONTRAST);
    oled_write_cmd(0xCF);
    oled_write_cmd(OLED_CMD_DISPLAY_NORMAL);
    oled_write_cmd(OLED_CMD_SET_PRECHARGE);
    oled_write_cmd(0xF1);
    oled_write_cmd(OLED_CMD_SET_VCOM_DETECT);
    oled_write_cmd(0x40);
    oled_write_cmd(OLED_CMD_CHARGE_PUMP);
    oled_write_cmd(0x14);
    oled_write_cmd(OLED_CMD_DEACTIVATE_SCROLL);
    oled_write_cmd(OLED_CMD_DISPLAY_ON);
    
    // Clear display
    for (int page = 0; page < 8; page++) {
        oled_write_cmd(0xB0 + page); // Set page
        oled_write_cmd(0x00); // Lower column
        oled_write_cmd(0x10); // Upper column
        
        uint8_t zeros[128] = {0};
        oled_write_data(zeros, 128);
    }
}

// Simple 5x7 font for basic text display
static void oled_draw_text(int x, int y, const char* text)
{
    // This is a simplified version - for full implementation you'd need a font table
    // For now, just clear and show basic status
    int page = y / 8;
    oled_write_cmd(0xB0 + page);
    oled_write_cmd(0x00 + (x & 0x0F));
    oled_write_cmd(0x10 + ((x >> 4) & 0x0F));
}

static void oled_update_display(void)
{
    // Clear display
    for (int page = 0; page < 8; page++) {
        oled_write_cmd(0xB0 + page);
        oled_write_cmd(0x00);
        oled_write_cmd(0x10);
        uint8_t zeros[128] = {0};
        oled_write_data(zeros, 128);
    }
    
    // For simplicity, we'll just log to console
    // Full OLED text rendering would require a font library
    ESP_LOGI(TAG, "Display Update:");
    ESP_LOGI(TAG, "  BT: %s", bt_connected ? "Connected" : "Waiting...");
    ESP_LOGI(TAG, "  GPS: %s (%d sats)", gps_data.gps_fixed ? "FIX" : "Searching", gps_data.satellites);
    if (gps_data.latitude[0]) {
        ESP_LOGI(TAG, "  Lat: %s", gps_data.latitude);
    }
    if (gps_data.longitude[0]) {
        ESP_LOGI(TAG, "  Lon: %s", gps_data.longitude);
    }
    if (gps_data.altitude[0]) {
        ESP_LOGI(TAG, "  Alt: %sm", gps_data.altitude);
    }
}

// Parse NMEA sentences
static void parse_nmea(const char *sentence)
{
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        // Parse GGA sentence
        char *token;
        char *buf = strdup(sentence);
        int field = 0;
        
        token = strtok(buf, ",");
        while (token != NULL) {
            field++;
            if (field == 7) { // Quality
                int quality = atoi(token);
                gps_data.gps_fixed = (quality > 0);
            } else if (field == 8) { // Satellites
                gps_data.satellites = atoi(token);
            } else if (field == 10) { // Altitude
                strncpy(gps_data.altitude, token, sizeof(gps_data.altitude) - 1);
            }
            token = strtok(NULL, ",");
        }
        free(buf);
    }
    else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        // Parse RMC sentence for position
        char *token;
        char *buf = strdup(sentence);
        int field = 0;

        token = strtok(buf, ",");
        while (token != NULL) {
            field++;
            if (field == 4) { // Latitude
                strncpy(gps_data.latitude, token, sizeof(gps_data.latitude) - 2);
                gps_data.latitude[sizeof(gps_data.latitude) - 2] = '\0';
            } else if (field == 5 && strlen(gps_data.latitude) < sizeof(gps_data.latitude) - 1) { // N/S
                size_t len = strlen(gps_data.latitude);
                gps_data.latitude[len] = token[0];
                gps_data.latitude[len + 1] = '\0';
            } else if (field == 6) { // Longitude
                strncpy(gps_data.longitude, token, sizeof(gps_data.longitude) - 2);
                gps_data.longitude[sizeof(gps_data.longitude) - 2] = '\0';
            } else if (field == 7 && strlen(gps_data.longitude) < sizeof(gps_data.longitude) - 1) { // E/W
                size_t len = strlen(gps_data.longitude);
                gps_data.longitude[len] = token[0];
                gps_data.longitude[len + 1] = '\0';
            }
            token = strtok(NULL, ",");
        }
        free(buf);
    }
}

// Bluetooth SPP callbacks
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP initialized");
        esp_bt_dev_set_device_name(BT_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        break;
        
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "SPP client connected");
        spp_handle = param->srv_open.handle;
        bt_connected = true;
        break;
        
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "SPP connection closed");
        bt_connected = false;
        break;
        
    case ESP_SPP_DATA_IND_EVT:
        // Received data from client (not expected for GPS)
        break;
        
    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(TAG, "Discovery state changed");
        break;
    default:
        break;
    }
}

// GPS UART task
static void gps_task(void *arg)
{
    static char nmea_buffer[256];
    static int nmea_idx = 0;
    uint8_t data[128];
    
    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        
        for (int i = 0; i < len; i++) {
            // Forward to Bluetooth if connected
            if (bt_connected && spp_handle) {
                esp_spp_write(spp_handle, 1, &data[i]);
            }
            
            // Build NMEA sentence
            if (data[i] == '$') {
                nmea_idx = 0;
                nmea_buffer[nmea_idx++] = data[i];
            } else if (data[i] == '\n' && nmea_idx > 0) {
                nmea_buffer[nmea_idx] = '\0';
                parse_nmea(nmea_buffer);
                nmea_idx = 0;
            } else if (nmea_idx < sizeof(nmea_buffer) - 1) {
                nmea_buffer[nmea_idx++] = data[i];
            }
        }
    }
}

// Display update task
static void display_task(void *arg)
{
    while (1) {
        oled_update_display();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ForeFlight Bluetooth GPS Starting...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize I2C and OLED
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "Initializing OLED...");
    oled_init();
    
    // Initialize UART for GPS
    ESP_LOGI(TAG, "Initializing GPS UART...");
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Initialize Bluetooth
    ESP_LOGI(TAG, "Initializing Bluetooth...");
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_cb));
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
    ESP_ERROR_CHECK(esp_spp_init(ESP_SPP_MODE_CB));
    
    // Create tasks
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "System ready!");
}
