/*
 * ForeFlight Bluetooth GPS - ESP-IDF Version (BLE)
 * ESP32 + M8N GPS + OLED Display
 *
 * Uses BLE Nordic UART Service (NUS) for iOS/ForeFlight compatibility
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
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "nvs_flash.h"

// Configuration
#define GPS_UART_NUM        UART_NUM_2
#define GPS_RX_PIN          16
#define GPS_TX_PIN          17
#define GPS_BAUD_RATE       115200
#define GPS_BUF_SIZE        1024

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define OLED_ADDRESS        0x3C

#define BLE_DEVICE_NAME     "ForeFlight GPS"
#define GATTS_APP_ID        0
#define BLE_MTU_SIZE        247

// Nordic UART Service UUIDs
// NUS Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
// NUS TX Char: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (Notify - ESP32 sends to phone)
// NUS RX Char: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Write - phone sends to ESP32)

static const uint8_t nus_service_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

static const uint8_t nus_tx_char_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

static const uint8_t nus_rx_char_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

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
static bool bt_connected = false;

// BLE state
static uint16_t ble_gatts_if = ESP_GATT_IF_NONE;
static uint16_t ble_conn_id = 0;
static uint16_t ble_tx_handle = 0;
static bool ble_notifications_enabled = false;

// GATT database handles
enum {
    IDX_SVC,
    IDX_TX_CHAR,
    IDX_TX_VAL,
    IDX_TX_CCC,
    IDX_RX_CHAR,
    IDX_RX_VAL,
    IDX_NB,
};

static uint16_t ble_handle_table[IDX_NB];

// BLE advertising data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = (uint8_t *)nus_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GATT service database
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t char_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t ccc_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static uint8_t ccc_val[2] = {0x00, 0x00};

static const esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    // Service Declaration
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
        ESP_GATT_PERM_READ, 16, sizeof(nus_service_uuid), (uint8_t *)nus_service_uuid}},

    // TX Characteristic Declaration (ESP32 -> Phone, Notify)
    [IDX_TX_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declaration_uuid,
        ESP_GATT_PERM_READ, 1, 1, (uint8_t *)&char_prop_notify}},

    // TX Characteristic Value
    [IDX_TX_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)nus_tx_char_uuid,
        0, BLE_MTU_SIZE, 0, NULL}},

    // TX Client Characteristic Configuration (for notifications)
    [IDX_TX_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ccc_uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 2, sizeof(ccc_val), (uint8_t *)ccc_val}},

    // RX Characteristic Declaration (Phone -> ESP32, Write)
    [IDX_RX_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declaration_uuid,
        ESP_GATT_PERM_READ, 1, 1, (uint8_t *)&char_prop_write}},

    // RX Characteristic Value
    [IDX_RX_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)nus_rx_char_uuid,
        ESP_GATT_PERM_WRITE, BLE_MTU_SIZE, 0, NULL}},
};

// ==================== I2C / OLED Functions ====================

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
    i2c_master_write_byte(i2c_cmd, 0x00, true);
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
    i2c_master_write_byte(i2c_cmd, 0x40, true);
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

    for (int page = 0; page < 8; page++) {
        oled_write_cmd(0xB0 + page);
        oled_write_cmd(0x00);
        oled_write_cmd(0x10);
        uint8_t zeros[128] = {0};
        oled_write_data(zeros, 128);
    }
}

// 5x7 font table (ASCII 32-126)
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x5F,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14}, // #
    {0x24,0x2A,0x7F,0x2A,0x12}, // $
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x05,0x03,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00}, // )
    {0x08,0x2A,0x1C,0x2A,0x08}, // *
    {0x08,0x08,0x3E,0x08,0x08}, // +
    {0x00,0x50,0x30,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x60,0x60,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02}, // /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x36,0x36,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00}, // ;
    {0x00,0x08,0x14,0x22,0x41}, // <
    {0x14,0x14,0x14,0x14,0x14}, // =
    {0x41,0x22,0x14,0x08,0x00}, // >
    {0x02,0x01,0x51,0x09,0x06}, // ?
    {0x32,0x49,0x79,0x41,0x3E}, // @
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x01,0x01}, // F
    {0x3E,0x41,0x41,0x51,0x32}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x04,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x7F,0x20,0x18,0x20,0x7F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x03,0x04,0x78,0x04,0x03}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x00,0x00,0x7F,0x41,0x41}, // [
    {0x02,0x04,0x08,0x10,0x20}, // backslash
    {0x41,0x41,0x7F,0x00,0x00}, // ]
    {0x04,0x02,0x01,0x02,0x04}, // ^
    {0x40,0x40,0x40,0x40,0x40}, // _
    {0x00,0x01,0x02,0x04,0x00}, // `
    {0x20,0x54,0x54,0x54,0x78}, // a
    {0x7F,0x48,0x44,0x44,0x38}, // b
    {0x38,0x44,0x44,0x44,0x20}, // c
    {0x38,0x44,0x44,0x48,0x7F}, // d
    {0x38,0x54,0x54,0x54,0x18}, // e
    {0x08,0x7E,0x09,0x01,0x02}, // f
    {0x08,0x54,0x54,0x54,0x3C}, // g
    {0x7F,0x08,0x04,0x04,0x78}, // h
    {0x00,0x44,0x7D,0x40,0x00}, // i
    {0x20,0x40,0x44,0x3D,0x00}, // j
    {0x00,0x7F,0x10,0x28,0x44}, // k
    {0x00,0x41,0x7F,0x40,0x00}, // l
    {0x7C,0x04,0x18,0x04,0x78}, // m
    {0x7C,0x08,0x04,0x04,0x78}, // n
    {0x38,0x44,0x44,0x44,0x38}, // o
    {0x7C,0x14,0x14,0x14,0x08}, // p
    {0x08,0x14,0x14,0x18,0x7C}, // q
    {0x7C,0x08,0x04,0x04,0x08}, // r
    {0x48,0x54,0x54,0x54,0x20}, // s
    {0x04,0x3F,0x44,0x40,0x20}, // t
    {0x3C,0x40,0x40,0x20,0x7C}, // u
    {0x1C,0x20,0x40,0x20,0x1C}, // v
    {0x3C,0x40,0x30,0x40,0x3C}, // w
    {0x44,0x28,0x10,0x28,0x44}, // x
    {0x0C,0x50,0x50,0x50,0x3C}, // y
    {0x44,0x64,0x54,0x4C,0x44}, // z
    {0x00,0x08,0x36,0x41,0x00}, // {
    {0x00,0x00,0x7F,0x00,0x00}, // |
    {0x00,0x41,0x36,0x08,0x00}, // }
    {0x08,0x08,0x2A,0x1C,0x08}, // ~
};

static void oled_draw_text(int page, int col, const char* text)
{
    oled_write_cmd(0xB0 + page);
    oled_write_cmd(0x00 + (col & 0x0F));
    oled_write_cmd(0x10 + ((col >> 4) & 0x0F));

    while (*text) {
        int idx = *text - 32;
        if (idx >= 0 && idx < 95) {
            oled_write_data((uint8_t *)font5x7[idx], 5);
            uint8_t space = 0x00;
            oled_write_data(&space, 1);
        }
        text++;
    }
}

static void oled_update_display(void)
{
    for (int page = 0; page < 8; page++) {
        oled_write_cmd(0xB0 + page);
        oled_write_cmd(0x00);
        oled_write_cmd(0x10);
        uint8_t zeros[128] = {0};
        oled_write_data(zeros, 128);
    }

    char line[22];

    oled_draw_text(0, 0, "ForeFlight GPS");

    snprintf(line, sizeof(line), "BT: %s", bt_connected ? "Connected" : "Waiting...");
    oled_draw_text(1, 0, line);

    snprintf(line, sizeof(line), "GPS: %s", gps_data.gps_fixed ? "FIX" : "Searching");
    oled_draw_text(2, 0, line);

    snprintf(line, sizeof(line), "Sats: %d", gps_data.satellites);
    oled_draw_text(3, 0, line);

    if (gps_data.altitude[0]) {
        snprintf(line, sizeof(line), "Alt: %sm", gps_data.altitude);
        oled_draw_text(4, 0, line);
    }

    if (gps_data.latitude[0]) {
        snprintf(line, sizeof(line), "Lat: %s", gps_data.latitude);
        oled_draw_text(5, 0, line);
    }

    if (gps_data.longitude[0]) {
        snprintf(line, sizeof(line), "Lon: %s", gps_data.longitude);
        oled_draw_text(6, 0, line);
    }
}

// ==================== NMEA Parser ====================

static void parse_nmea(const char *sentence)
{
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        char *token;
        char *buf = strdup(sentence);
        int field = 0;

        token = strtok(buf, ",");
        while (token != NULL) {
            field++;
            if (field == 7) {
                int quality = atoi(token);
                gps_data.gps_fixed = (quality > 0);
            } else if (field == 8) {
                gps_data.satellites = atoi(token);
            } else if (field == 10) {
                strncpy(gps_data.altitude, token, sizeof(gps_data.altitude) - 1);
            }
            token = strtok(NULL, ",");
        }
        free(buf);
    }
    else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        char *token;
        char *buf = strdup(sentence);
        int field = 0;

        token = strtok(buf, ",");
        while (token != NULL) {
            field++;
            if (field == 4) {
                strncpy(gps_data.latitude, token, sizeof(gps_data.latitude) - 1);
            } else if (field == 5) {
                strncat(gps_data.latitude, token, 1);
            } else if (field == 6) {
                strncpy(gps_data.longitude, token, sizeof(gps_data.longitude) - 1);
            } else if (field == 7) {
                strncat(gps_data.longitude, token, 1);
            }
            token = strtok(NULL, ",");
        }
        free(buf);
    }
}

// ==================== GPS UBX Configuration ====================

static void gps_send_ubx(const uint8_t *msg, size_t len)
{
    uart_write_bytes(GPS_UART_NUM, (const char *)msg, len);
    uart_wait_tx_done(GPS_UART_NUM, pdMS_TO_TICKS(100));
}

static void gps_configure(void)
{
    ESP_LOGI(TAG, "Configuring GPS for NMEA output...");

    uint8_t cfg_prt[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
        0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00,
        0x00, 0xC2, 0x01, 0x00,
        0x07, 0x00, 0x02, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00
    };
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < (int)sizeof(cfg_prt) - 2; i++) {
        ck_a += cfg_prt[i];
        ck_b += ck_a;
    }
    cfg_prt[sizeof(cfg_prt) - 2] = ck_a;
    cfg_prt[sizeof(cfg_prt) - 1] = ck_b;
    gps_send_ubx(cfg_prt, sizeof(cfg_prt));
    vTaskDelay(pdMS_TO_TICKS(200));

    uart_flush(GPS_UART_NUM);
    ESP_LOGI(TAG, "GPS configured for NMEA output");
}

// ==================== BLE Send ====================

static void ble_send_data(const uint8_t *data, size_t len)
{
    if (!bt_connected || !ble_notifications_enabled || ble_gatts_if == ESP_GATT_IF_NONE) {
        return;
    }

    // BLE can send max ~20 bytes per notification (or MTU-3)
    // Send in chunks
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > 20) chunk = 20;
        esp_ble_gatts_send_indicate(ble_gatts_if, ble_conn_id,
            ble_handle_table[IDX_TX_VAL], chunk, (uint8_t *)data + offset, false);
        offset += chunk;
        if (offset < len) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ==================== BLE Callbacks ====================

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "BLE advertising started");
        }
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "BLE GATT registered");
        ble_gatts_if = gatts_if;
        esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NB, 0);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK && param->add_attr_tab.num_handle == IDX_NB) {
            memcpy(ble_handle_table, param->add_attr_tab.handles, sizeof(ble_handle_table));
            ble_tx_handle = ble_handle_table[IDX_TX_VAL];
            esp_ble_gatts_start_service(ble_handle_table[IDX_SVC]);
            ESP_LOGI(TAG, "BLE NUS service started");
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "BLE client connected");
        ble_conn_id = param->connect.conn_id;
        bt_connected = true;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "BLE client disconnected");
        bt_connected = false;
        ble_notifications_enabled = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == ble_handle_table[IDX_TX_CCC]) {
            if (param->write.len == 2) {
                uint16_t descr_value = (param->write.value[1] << 8) | param->write.value[0];
                ble_notifications_enabled = (descr_value == 0x0001);
                ESP_LOGI(TAG, "BLE notifications %s", ble_notifications_enabled ? "enabled" : "disabled");
            }
        }
        break;

    default:
        break;
    }
}

// ==================== Tasks ====================

static void gps_task(void *arg)
{
    static char nmea_buffer[256];
    static int nmea_idx = 0;
    uint8_t data[128];

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));

        // Forward raw GPS data to BLE
        if (len > 0 && bt_connected && ble_notifications_enabled) {
            ble_send_data(data, len);
        }

        for (int i = 0; i < len; i++) {
            if (data[i] == '$') {
                nmea_idx = 0;
                nmea_buffer[nmea_idx++] = data[i];
            } else if (data[i] == '\n' && nmea_idx > 0) {
                nmea_buffer[nmea_idx] = '\0';
                parse_nmea(nmea_buffer);
                nmea_idx = 0;
            } else if (nmea_idx < (int)sizeof(nmea_buffer) - 1) {
                nmea_buffer[nmea_idx++] = data[i];
            }
        }
    }
}

static void display_task(void *arg)
{
    while (1) {
        oled_update_display();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ==================== Main ====================

void app_main(void)
{
    ESP_LOGI(TAG, "ForeFlight Bluetooth GPS Starting (BLE)...");

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

    // Scan I2C bus to find OLED address
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, ">>> I2C device found at address 0x%02X", addr);
        }
    }

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

    // Configure GPS module to output NMEA
    gps_configure();

    // Initialize BLE
    ESP_LOGI(TAG, "Initializing BLE...");
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATTS_APP_ID));

    // Create tasks
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "System ready!");
}
