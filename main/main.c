/*
 * ForeFlight WiFi GPS - ESP-IDF Version
 * ESP32 + M8N GPS + OLED Display
 *
 * Sends GPS data to ForeFlight via WiFi using GDL90 protocol.
 * ESP32 creates a WiFi access point, iPad connects to it,
 * and ForeFlight automatically receives GPS data via UDP broadcast.
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
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

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

#define WIFI_SSID           "ForeFlight GPS"
#define WIFI_CHANNEL        6
#define WIFI_MAX_CONN       4

#define GDL90_PORT          4000
#define GDL90_FLAG          0x7E
#define GDL90_ESCAPE        0x7D

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
    char latitude_str[20];
    char longitude_str[20];
    char altitude_str[16];
    double latitude;      // decimal degrees (+N, -S)
    double longitude;     // decimal degrees (+E, -W)
    double altitude_m;    // meters
    double speed_kt;      // ground speed in knots
    double track_deg;     // true track in degrees
} gps_data_t;

static gps_data_t gps_data = {0};
static int wifi_clients = 0;
static int udp_sock = -1;

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

    char line[32];

    oled_draw_text(0, 0, "ForeFlight GPS");

    snprintf(line, sizeof(line), "WiFi: %s", wifi_clients > 0 ? "Connected" : "Waiting...");
    oled_draw_text(1, 0, line);

    snprintf(line, sizeof(line), "GPS: %s", gps_data.gps_fixed ? "FIX" : "Searching");
    oled_draw_text(2, 0, line);

    snprintf(line, sizeof(line), "Sats: %d", gps_data.satellites);
    oled_draw_text(3, 0, line);

    if (gps_data.altitude_str[0]) {
        snprintf(line, sizeof(line), "Alt: %sm", gps_data.altitude_str);
        oled_draw_text(4, 0, line);
    }

    if (gps_data.latitude_str[0]) {
        snprintf(line, sizeof(line), "Lat: %s", gps_data.latitude_str);
        oled_draw_text(5, 0, line);
    }

    if (gps_data.longitude_str[0]) {
        snprintf(line, sizeof(line), "Lon: %s", gps_data.longitude_str);
        oled_draw_text(6, 0, line);
    }
}

// ==================== NMEA Parser ====================

static double nmea_to_dd(const char *coord, char dir)
{
    if (!coord || !coord[0]) return 0.0;
    double raw = atof(coord);
    int deg = (int)(raw / 100);
    double min = raw - deg * 100;
    double dd = deg + min / 60.0;
    if (dir == 'S' || dir == 'W') dd = -dd;
    return dd;
}

static void parse_nmea(const char *sentence)
{
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        char *buf = strdup(sentence);
        char *token = strtok(buf, ",");
        int field = 0;
        char lat[16] = "", lon[16] = "";
        char lat_d = 'N', lon_d = 'E';

        while (token != NULL) {
            field++;
            switch (field) {
                case 3: strncpy(lat, token, sizeof(lat) - 1); break;
                case 4: if (token[0]) lat_d = token[0]; break;
                case 5: strncpy(lon, token, sizeof(lon) - 1); break;
                case 6: if (token[0]) lon_d = token[0]; break;
                case 7: gps_data.gps_fixed = (atoi(token) > 0); break;
                case 8: gps_data.satellites = atoi(token); break;
                case 10:
                    strncpy(gps_data.altitude_str, token, sizeof(gps_data.altitude_str) - 1);
                    gps_data.altitude_m = atof(token);
                    break;
            }
            token = strtok(NULL, ",");
        }

        if (lat[0]) {
            gps_data.latitude = nmea_to_dd(lat, lat_d);
            snprintf(gps_data.latitude_str, sizeof(gps_data.latitude_str), "%s%c", lat, lat_d);
        }
        if (lon[0]) {
            gps_data.longitude = nmea_to_dd(lon, lon_d);
            snprintf(gps_data.longitude_str, sizeof(gps_data.longitude_str), "%s%c", lon, lon_d);
        }

        free(buf);
    }
    else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        char *buf = strdup(sentence);
        char *token = strtok(buf, ",");
        int field = 0;

        while (token != NULL) {
            field++;
            switch (field) {
                case 8: gps_data.speed_kt = atof(token); break;
                case 9: gps_data.track_deg = atof(token); break;
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
    ESP_LOGI(TAG, "Configuring GPS for NMEA output and WAAS/SBAS...");

    // CFG-PRT: Configure UART for NMEA output
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

    // CFG-SBAS: Enable WAAS/SBAS for accuracy improvement (~1m)
    uint8_t cfg_sbas[] = {
        0xB5, 0x62, 0x06, 0x16, 0x08, 0x00,
        0x01,                   // Mode: 1 = enabled
        0x07,                   // Usage: 7 = use for GPS correction
        0x03,                   // maxSBAS: 3 satellites
        0x00,                   // flags
        0x00, 0x00, 0x00, 0x00, // padding
        0x00, 0x00              // checksum (will be calculated)
    };
    ck_a = 0;
    ck_b = 0;
    for (int i = 2; i < (int)sizeof(cfg_sbas) - 2; i++) {
        ck_a += cfg_sbas[i];
        ck_b += ck_a;
    }
    cfg_sbas[sizeof(cfg_sbas) - 2] = ck_a;
    cfg_sbas[sizeof(cfg_sbas) - 1] = ck_b;
    gps_send_ubx(cfg_sbas, sizeof(cfg_sbas));
    vTaskDelay(pdMS_TO_TICKS(200));

    uart_flush(GPS_UART_NUM);
    ESP_LOGI(TAG, "GPS configured for NMEA output with WAAS/SBAS enabled");
}

// ==================== GDL90 Protocol ====================

static uint16_t gdl90_crc_table[256];

static void gdl90_crc_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint16_t crc = (uint16_t)(i << 8);
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
        gdl90_crc_table[i] = crc;
    }
}

static uint16_t gdl90_crc(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = gdl90_crc_table[crc >> 8] ^ (crc << 8) ^ data[i];
    }
    return crc;
}

static size_t gdl90_frame(const uint8_t *msg, size_t msg_len, uint8_t *out)
{
    uint16_t crc = gdl90_crc(msg, msg_len);
    size_t idx = 0;

    out[idx++] = GDL90_FLAG;

    // Byte-stuff message data
    for (size_t i = 0; i < msg_len; i++) {
        if (msg[i] == GDL90_FLAG || msg[i] == GDL90_ESCAPE) {
            out[idx++] = GDL90_ESCAPE;
            out[idx++] = msg[i] ^ 0x20;
        } else {
            out[idx++] = msg[i];
        }
    }

    // CRC low byte (byte-stuffed)
    uint8_t crc_lo = crc & 0xFF;
    if (crc_lo == GDL90_FLAG || crc_lo == GDL90_ESCAPE) {
        out[idx++] = GDL90_ESCAPE;
        out[idx++] = crc_lo ^ 0x20;
    } else {
        out[idx++] = crc_lo;
    }

    // CRC high byte (byte-stuffed)
    uint8_t crc_hi = (crc >> 8) & 0xFF;
    if (crc_hi == GDL90_FLAG || crc_hi == GDL90_ESCAPE) {
        out[idx++] = GDL90_ESCAPE;
        out[idx++] = crc_hi ^ 0x20;
    } else {
        out[idx++] = crc_hi;
    }

    out[idx++] = GDL90_FLAG;
    return idx;
}

static void gdl90_send(const uint8_t *data, size_t len)
{
    if (udp_sock < 0) return;

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(GDL90_PORT),
    };
    inet_aton("192.168.4.255", &dest.sin_addr);

    sendto(udp_sock, data, len, 0, (struct sockaddr *)&dest, sizeof(dest));
}

// GDL90 Heartbeat (Message ID 0) - sent every second
static void gdl90_send_heartbeat(void)
{
    uint8_t msg[7] = {
        0x00,  // Message ID
        (uint8_t)(gps_data.gps_fixed ? 0x81 : 0x01),  // Status 1: GPS valid + UAT init
        0x00,  // Status 2
        0x00, 0x00,  // Timestamp
        0x00, 0x00,  // Message counts
    };
    uint8_t frame[32];
    size_t len = gdl90_frame(msg, 7, frame);
    gdl90_send(frame, len);
}

// GDL90 Ownship Report (Message ID 10) - GPS position
static void gdl90_send_ownship(void)
{
    uint8_t msg[28];
    memset(msg, 0, sizeof(msg));

    msg[0] = 0x0A;  // Message ID: Ownship Report
    msg[1] = 0x01;  // Address type: self-assigned

    // Participant address
    msg[2] = 0x00;
    msg[3] = 0x00;
    msg[4] = 0x01;

    // Latitude (24-bit signed, semicircles)
    int32_t lat_enc = (int32_t)(gps_data.latitude / 180.0 * (1 << 23));
    msg[5] = (lat_enc >> 16) & 0xFF;
    msg[6] = (lat_enc >> 8) & 0xFF;
    msg[7] = lat_enc & 0xFF;

    // Longitude (24-bit signed, semicircles)
    int32_t lon_enc = (int32_t)(gps_data.longitude / 180.0 * (1 << 23));
    msg[8] = (lon_enc >> 16) & 0xFF;
    msg[9] = (lon_enc >> 8) & 0xFF;
    msg[10] = lon_enc & 0xFF;

    // Altitude (12-bit, 25ft increments from -1000ft)
    double alt_ft = gps_data.altitude_m * 3.28084;
    int alt_enc = (int)((alt_ft + 1000.0) / 25.0);
    if (alt_enc < 0) alt_enc = 0;
    if (alt_enc > 0xFFE) alt_enc = 0xFFE;

    // Misc: airborne + true track + updated
    uint8_t misc = 0x0B;
    msg[11] = (alt_enc >> 4) & 0xFF;
    msg[12] = ((alt_enc & 0x0F) << 4) | misc;

    // NIC=8, NACp=8
    msg[13] = 0x88;

    // Horizontal velocity (12-bit, knots)
    int hvel = (int)gps_data.speed_kt;
    if (hvel > 0xFFE) hvel = 0xFFE;

    // Vertical velocity (12-bit signed, 64fpm increments) - unknown
    int vvel = 0x800;

    msg[14] = (hvel >> 4) & 0xFF;
    msg[15] = ((hvel & 0x0F) << 4) | ((vvel >> 8) & 0x0F);
    msg[16] = vvel & 0xFF;

    // Track/Heading
    msg[17] = (uint8_t)(gps_data.track_deg / 360.0 * 256.0);

    // Emitter category: light aircraft
    msg[18] = 0x01;

    // Callsign (8 bytes, space-padded)
    memcpy(msg + 19, "FFGPS   ", 8);

    // Emergency/Priority + spare
    msg[27] = 0x00;

    uint8_t frame[96];
    size_t len = gdl90_frame(msg, 28, frame);
    gdl90_send(frame, len);
}

// GDL90 Ownship Geometric Altitude (Message ID 11)
static void gdl90_send_geo_alt(void)
{
    uint8_t msg[5];
    msg[0] = 0x0B;  // Message ID

    // Geometric altitude in 5-foot increments
    double alt_ft = gps_data.altitude_m * 3.28084;
    int16_t geo_alt = (int16_t)(alt_ft / 5.0);
    msg[1] = (geo_alt >> 8) & 0xFF;
    msg[2] = geo_alt & 0xFF;

    // Vertical figure of merit (meters)
    msg[3] = 0x00;
    msg[4] = 0x0A;  // 10 meters

    uint8_t frame[32];
    size_t len = gdl90_frame(msg, 5, frame);
    gdl90_send(frame, len);
}

// ForeFlight extended ID message (Message ID 0x65)
static void gdl90_send_ff_id(void)
{
    uint8_t msg[31];
    memset(msg, 0, sizeof(msg));

    msg[0] = 0x65;  // ForeFlight message ID
    msg[1] = 0x00;  // Sub-type: device ID
    msg[2] = 0x01;  // Version

    // Device serial (8 bytes, space-padded)
    memcpy(msg + 3, "ESPGPS01", 8);

    // Device long name (16 bytes, space-padded)
    memcpy(msg + 11, "ForeFlight GPS  ", 16);

    // Capabilities (4 bytes, big-endian)
    msg[27] = 0x00;
    msg[28] = 0x00;
    msg[29] = 0x00;
    msg[30] = 0x01;  // Bit 0: WAAS GPS

    uint8_t frame[96];
    size_t len = gdl90_frame(msg, 31, frame);
    gdl90_send(frame, len);
}

// ==================== WiFi ====================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_clients++;
        ESP_LOGI(TAG, "WiFi client connected (total: %d)", wifi_clients);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        if (wifi_clients > 0) wifi_clients--;
        ESP_LOGI(TAG, "WiFi client disconnected (total: %d)", wifi_clients);
    }
}

static void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = sizeof(WIFI_SSID) - 1,
            .channel = WIFI_CHANNEL,
            .max_connection = WIFI_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started - SSID: %s", WIFI_SSID);
}

static void udp_init(void)
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return;
    }

    int broadcast = 1;
    setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    ESP_LOGI(TAG, "UDP socket ready (port %d)", GDL90_PORT);
}

// ==================== Tasks ====================

static void gps_task(void *arg)
{
    static char nmea_buffer[256];
    static int nmea_idx = 0;
    uint8_t data[128];

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));

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

static void gdl90_task(void *arg)
{
    gdl90_crc_init();

    while (1) {
        // Always send heartbeat
        gdl90_send_heartbeat();

        // Send position if we have a fix
        if (gps_data.gps_fixed) {
            gdl90_send_ownship();
            gdl90_send_geo_alt();
        }

        // Send ForeFlight device ID
        gdl90_send_ff_id();

        vTaskDelay(pdMS_TO_TICKS(1000));
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
    ESP_LOGI(TAG, "ForeFlight WiFi GPS Starting...");

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

    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
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

    // Initialize WiFi AP
    ESP_LOGI(TAG, "Starting WiFi AP...");
    wifi_init_ap();

    // Initialize UDP socket
    udp_init();

    // Create tasks
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(gdl90_task, "gdl90_task", 4096, NULL, 4, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "System ready! Connect iPad to WiFi: %s", WIFI_SSID);
}
