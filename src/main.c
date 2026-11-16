#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "driver/i2c_master.h"
#include "driver/uart.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_sleep.h"
#include "esp_timer.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define I2C_PORT                0
#define I2C_SDA_GPIO            1
#define I2C_SCL_GPIO            2
#define I2C_FREQ_HZ             100000

#define OLED_ADDR               0x3C
#define BME280_ADDR             0x76

#define UART_PORT UART_NUM_0
#define UART_BAUD 115200

#define SAMPLE_PERIOD_MS        1000
#define FLASH_LOG_PERIOD_SEC    10
#define FLASH_SECTOR_SIZE       4096

#define SENSOR_QUEUE_LEN        1
#define FLASH_QUEUE_LEN         8

#define BIT_SENSOR_DONE         BIT0
#define BIT_UART_DONE           BIT1
#define BIT_DISPLAY_DONE        BIT2
#define CYCLE_DONE_BITS         (BIT_SENSOR_DONE | BIT_UART_DONE | BIT_DISPLAY_DONE)

#define BME280_CHIP_ID_REG      0xD0
#define BME280_CHIP_ID          0x60
#define BME280_RESET_REG        0xE0
#define BME280_CTRL_HUM_REG     0xF2
#define BME280_STATUS_REG       0xF3
#define BME280_CTRL_MEAS_REG    0xF4
#define BME280_CONFIG_REG       0xF5
#define BME280_DATA_REG         0xF7

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uint32_t sample_id;
    int64_t timestamp_ms;
    float temperature_c;
    float humidity_percent;
    float pressure_hpa;
    bool valid;
} sensor_data_t;

typedef struct __attribute__((packed))
{
    uint32_t sample_id;
    uint32_t timestamp_ms_low;
    float temperature_c;
    float humidity_percent;
    float pressure_hpa;
} log_record_t;

_Static_assert(sizeof(log_record_t) == 20, "Unexpected log_record_t size");

typedef enum
{
    FLASH_MSG_SAMPLE = 0,
    FLASH_MSG_FLUSH
} flash_msg_type_t;

typedef struct
{
    flash_msg_type_t type;
    log_record_t record;
} flash_message_t;

typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
    int32_t t_fine;
} bme280_calib_t;

/* -------------------------------------------------------------------------- */
/* Globals                                                                    */
/* -------------------------------------------------------------------------- */

static const char *TAG = "LOGGER";

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t oled_handle;
static i2c_master_dev_handle_t bme_handle;

static SemaphoreHandle_t i2c_mutex;
static QueueHandle_t uart_queue;
static QueueHandle_t display_queue;
static QueueHandle_t flash_queue;
static EventGroupHandle_t cycle_events;
static TaskHandle_t sensor_task_handle;
static const esp_partition_t *log_partition;

static volatile bool logging_enabled = true;
static volatile bool flash_full = false;
static uint32_t sample_counter = 0;
static bme280_calib_t bme_calib;

/* -------------------------------------------------------------------------- */
/* OLED                                                                       */
/* -------------------------------------------------------------------------- */

static const uint8_t font5x8[59][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14}, {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x01,0x01}, {0x3E,0x41,0x41,0x51,0x32},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03}, {0x61,0x51,0x49,0x45,0x43}
};

static void oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_transmit(oled_handle, buf, 2, -1);
}

static void oled_data(const uint8_t *data, size_t len)
{
    uint8_t buf[129];
    if (len > 128) len = 128;
    buf[0] = 0x40;
    memcpy(buf + 1, data, len);
    i2c_master_transmit(oled_handle, buf, len + 1, -1);
}

static void oled_init(void)
{
    uint8_t cmds[] = {
        0xAE, 0x20, 0x00, 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07,
        0x81, 0x7F, 0xA1, 0xC8, 0xA6, 0xA8, 0x3F, 0xD3, 0x00,
        0xD5, 0x80, 0xD9, 0x22, 0xDA, 0x12, 0xDB, 0x20,
        0x8D, 0x14, 0xAF
    };

    for (size_t i = 0; i < sizeof(cmds); ++i) oled_cmd(cmds[i]);
}

static void oled_print(uint8_t page, uint8_t col, const char *str)
{
    oled_cmd(0xB0 + page);
    oled_cmd(0x00 + (col & 0x0F));
    oled_cmd(0x10 + ((col >> 4) & 0x0F));

    while (*str) {
        if (*str >= 32 && *str <= 90) oled_data(font5x8[(uint8_t)*str - 32], 5);
        const uint8_t space = 0x00;
        oled_data(&space, 1);
        ++str;
    }
}

static void oled_clear(void)
{
    uint8_t page_data[128];
    memset(page_data, 0x00, sizeof(page_data));

    uint8_t buf[129];
    buf[0] = 0x40;
    memcpy(buf + 1, page_data, 128);

    for (int i = 0; i < 8; ++i) {
        oled_cmd(0xB0 + i);
        oled_cmd(0x00);
        oled_cmd(0x10);
        i2c_master_transmit(oled_handle, buf, sizeof(buf), -1);
    }
}

static void oled_show_values(const sensor_data_t *data)
{
    char line[24];
    oled_clear();

    if (!data->valid) {
        oled_print(2, 0, "BME280 ERROR");
        return;
    }

    snprintf(line, sizeof(line), "TEMP: %.1F C", data->temperature_c);
    oled_print(0, 0, line);

    snprintf(line, sizeof(line), "HUM: %.1F %%", data->humidity_percent);
    oled_print(2, 0, line);

    snprintf(line, sizeof(line), "PRES: %.0F HPA", data->pressure_hpa);
    oled_print(4, 0, line);

    if (flash_full) {
        oled_print(6, 0, "LOG: FULL");
    } else {
        snprintf(line, sizeof(line), "LOG: %s", logging_enabled ? "ON" : "OFF");
        oled_print(6, 0, line);
    }
}

/* -------------------------------------------------------------------------- */
/* BME280                                                                     */
/* -------------------------------------------------------------------------- */

static uint16_t u16_le(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static int16_t s16_le(const uint8_t *p) { return (int16_t)u16_le(p); }

static esp_err_t bme_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(bme_handle, &reg, 1, data, len, 100);
}

static esp_err_t bme_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(bme_handle, data, sizeof(data), 100);
}

static esp_err_t bme280_read_calibration(void)
{
    uint8_t a[26];
    uint8_t h[7];

    ESP_RETURN_ON_ERROR(bme_read_regs(0x88, a, sizeof(a)), TAG, "Calibration block 1");
    ESP_RETURN_ON_ERROR(bme_read_regs(0xE1, h, sizeof(h)), TAG, "Calibration block 2");

    bme_calib.dig_T1 = u16_le(&a[0]);
    bme_calib.dig_T2 = s16_le(&a[2]);
    bme_calib.dig_T3 = s16_le(&a[4]);
    bme_calib.dig_P1 = u16_le(&a[6]);
    bme_calib.dig_P2 = s16_le(&a[8]);
    bme_calib.dig_P3 = s16_le(&a[10]);
    bme_calib.dig_P4 = s16_le(&a[12]);
    bme_calib.dig_P5 = s16_le(&a[14]);
    bme_calib.dig_P6 = s16_le(&a[16]);
    bme_calib.dig_P7 = s16_le(&a[18]);
    bme_calib.dig_P8 = s16_le(&a[20]);
    bme_calib.dig_P9 = s16_le(&a[22]);
    bme_calib.dig_H1 = a[25];
    bme_calib.dig_H2 = s16_le(&h[0]);
    bme_calib.dig_H3 = h[2];
    bme_calib.dig_H4 = (int16_t)((((int16_t)(int8_t)h[3]) << 4) | (h[4] & 0x0F));
    bme_calib.dig_H5 = (int16_t)((((int16_t)(int8_t)h[5]) << 4) | (h[4] >> 4));
    bme_calib.dig_H6 = (int8_t)h[6];

    return ESP_OK;
}

static esp_err_t bme280_init(void)
{
    uint8_t id = 0;

    ESP_LOGI(TAG, "BME: reading chip ID");

    esp_err_t err = bme_read_regs(
        BME280_CHIP_ID_REG,
        &id,
        1
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "BME: chip ID read failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }

    ESP_LOGI(
        TAG,
        "BME: chip ID = 0x%02X",
        id
    );

    if (id != BME280_CHIP_ID) {
        ESP_LOGE(
            TAG,
            "Unexpected BME280 ID: 0x%02X",
            id
        );

        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "BME: reset");

    err = bme_write_reg(
        BME280_RESET_REG,
        0xB6
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "BME reset failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "BME: reading calibration");

    err = bme280_read_calibration();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Calibration read failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }

    ESP_LOGI(TAG, "BME: configuring humidity");

    err = bme_write_reg(
        BME280_CTRL_HUM_REG,
        0x01
    );

    if (err != ESP_OK) {
        return err;
    }

    err = bme_write_reg(
        BME280_CONFIG_REG,
        0x00
    );

    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "BME280 ready");

    return ESP_OK;
}

static float bme280_compensate_temperature(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme_calib.dig_T1 << 1))) * ((int32_t)bme_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bme_calib.dig_T1))) >> 12) * ((int32_t)bme_calib.dig_T3)) >> 14;
    bme_calib.t_fine = var1 + var2;
    int32_t t = (bme_calib.t_fine * 5 + 128) >> 8;
    return (float)t / 100.0f;
}

static float bme280_compensate_pressure(int32_t adc_P)
{
    int64_t var1 = ((int64_t)bme_calib.t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bme_calib.dig_P6;
    var2 += (var1 * (int64_t)bme_calib.dig_P5) << 17;
    var2 += ((int64_t)bme_calib.dig_P4) << 35;
    var1 = ((var1 * var1 * (int64_t)bme_calib.dig_P3) >> 8) + ((var1 * (int64_t)bme_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)bme_calib.dig_P1)) >> 33;

    if (var1 == 0) return NAN;

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bme_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bme_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bme_calib.dig_P7) << 4);
    return (float)p / 25600.0f;
}

static float bme280_compensate_humidity(int32_t adc_H)
{
    int32_t v = bme_calib.t_fine - 76800;
    v = (((((adc_H << 14) - (((int32_t)bme_calib.dig_H4) << 20) - (((int32_t)bme_calib.dig_H5) * v)) + 16384) >> 15) *
         (((((((v * ((int32_t)bme_calib.dig_H6)) >> 10) * (((v * ((int32_t)bme_calib.dig_H3)) >> 11) + 32768)) >> 10) + 2097152) * ((int32_t)bme_calib.dig_H2) + 8192) >> 14));
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)bme_calib.dig_H1)) >> 4);
    if (v < 0) v = 0;
    if (v > 419430400) v = 419430400;
    return (float)(v >> 12) / 1024.0f;
}

static esp_err_t bme280_read(sensor_data_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    /* osrs_t=x1, osrs_p=x1, forced mode. */
    ESP_RETURN_ON_ERROR(bme_write_reg(BME280_CTRL_MEAS_REG, 0x25), TAG, "forced measurement");
    vTaskDelay(pdMS_TO_TICKS(15));

    uint8_t status = 0;
    bool measurement_done = false;

    for (int i = 0; i < 50; ++i)
    {
        ESP_RETURN_ON_ERROR(
            bme_read_regs(
                BME280_STATUS_REG,
                &status,
                1
            ),
            TAG,
            "BME280 status"
        );

        if ((status & 0x08) == 0)
        {
            measurement_done = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
}

if (!measurement_done)
{
    ESP_LOGE(TAG, "BME280 measurement timeout");
    return ESP_ERR_TIMEOUT;
}

    uint8_t raw[8];
    ESP_RETURN_ON_ERROR(bme_read_regs(BME280_DATA_REG, raw, sizeof(raw)), TAG, "sensor data");

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] << 8) | (int32_t)raw[7];

    out->temperature_c = bme280_compensate_temperature(adc_T);
    out->pressure_hpa = bme280_compensate_pressure(adc_P);
    out->humidity_percent = bme280_compensate_humidity(adc_H);
    out->valid = isfinite(out->temperature_c) && isfinite(out->pressure_hpa) && isfinite(out->humidity_percent);
    return out->valid ? ESP_OK : ESP_FAIL;
}

/* -------------------------------------------------------------------------- */
/* Flash                                                                      */
/* -------------------------------------------------------------------------- */

static bool sector_is_erased(size_t offset)
{
    uint8_t probe[32];
    if (esp_partition_read(log_partition, offset, probe, sizeof(probe)) != ESP_OK) return false;
    for (size_t i = 0; i < sizeof(probe); ++i) if (probe[i] != 0xFF) return false;
    return true;
}

static size_t find_first_free_sector(void)
{
    size_t sector_count = log_partition->size / FLASH_SECTOR_SIZE;
    for (size_t i = 0; i < sector_count; ++i) {
        size_t offset = i * FLASH_SECTOR_SIZE;
        if (sector_is_erased(offset)) return offset;
    }
    return log_partition->size;
}

static esp_err_t write_sector(size_t offset, const uint8_t *buffer)
{
    if ((offset + FLASH_SECTOR_SIZE) > log_partition->size) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(esp_partition_erase_range(log_partition, offset, FLASH_SECTOR_SIZE), TAG, "erase log sector");
    return esp_partition_write(log_partition, offset, buffer, FLASH_SECTOR_SIZE);
}

/* -------------------------------------------------------------------------- */
/* Tasks                                                                      */
/* -------------------------------------------------------------------------- */

static void sensor_task(void *arg)
{
    sensor_data_t data;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        memset(&data, 0, sizeof(data));
        data.sample_id = ++sample_counter;
        data.timestamp_ms = esp_timer_get_time() / 1000;

        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            esp_err_t err = bme280_read(&data);
            xSemaphoreGive(i2c_mutex);
            if (err != ESP_OK) data.valid = false;
        } else {
            data.valid = false;
        }

        xQueueOverwrite(uart_queue, &data);
        xQueueOverwrite(display_queue, &data);

        if (logging_enabled && !flash_full && data.valid &&
            (data.sample_id % FLASH_LOG_PERIOD_SEC) == 0) {
            flash_message_t msg = {
                .type = FLASH_MSG_SAMPLE,
                .record = {
                    .sample_id = data.sample_id,
                    .timestamp_ms_low = (uint32_t)(data.timestamp_ms & 0xFFFFFFFFu),
                    .temperature_c = data.temperature_c,
                    .humidity_percent = data.humidity_percent,
                    .pressure_hpa = data.pressure_hpa
                }
            };

            if (xQueueSend(flash_queue, &msg, pdMS_TO_TICKS(20)) != pdTRUE) {
                ESP_LOGW(TAG, "Flash queue full");
            }
        }

        xEventGroupSetBits(cycle_events, BIT_SENSOR_DONE);
    }
}

static void uart_handle_command(const char *command)
{
    if (strcmp(command, "status") == 0) {
        printf("STATUS logging=%s flash=%s samples=%lu\n",
               logging_enabled ? "ON" : "OFF",
               flash_full ? "FULL" : "OK",
               (unsigned long)sample_counter);
    } else if (strcmp(command, "log on") == 0) {
        if (flash_full) printf("ERR flash partition full\n");
        else { logging_enabled = true; printf("OK logging enabled\n"); }
    } else if (strcmp(command, "log off") == 0) {
        logging_enabled = false;
        printf("OK logging disabled\n");
    } else if (strcmp(command, "flush") == 0) {
        flash_message_t msg = {.type = FLASH_MSG_FLUSH};
        if (xQueueSend(flash_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) printf("OK flush requested\n");
        else printf("ERR flush queue timeout\n");
    } else {
        printf("ERR unknown command\n");
    }
}

static void uart_task(void *arg)
{
    sensor_data_t data;

    char command[32];
    size_t command_len = 0;

    while (true)
    {
        /* Print new sensor sample. */
        if (xQueueReceive(
                uart_queue,
                &data,
                pdMS_TO_TICKS(20)) == pdTRUE)
        {
            if (data.valid)
            {
                printf(
                    "#%lu T=%.2fC H=%.2f%% P=%.2fhPa\n",
                    (unsigned long)data.sample_id,
                    data.temperature_c,
                    data.humidity_percent,
                    data.pressure_hpa
                );
            }
            else
            {
                printf(
                    "#%lu BME280 ERROR\n",
                    (unsigned long)data.sample_id
                );
            }

            xEventGroupSetBits(
                cycle_events,
                BIT_UART_DONE
            );
        }

        /* Read UART commands. */
        uint8_t ch;

        int len = uart_read_bytes(
            UART_PORT,
            &ch,
            1,
            pdMS_TO_TICKS(10)
        );

        if (len > 0)
        {
            /* Local echo for debugging. */
            uart_write_bytes(
                UART_PORT,
                (const char *)&ch,
                1
            );

            if (ch == '\r' || ch == '\n')
            {
                if (command_len > 0)
                {
                    command[command_len] = '\0';

                    printf("\nCMD: [%s]\n", command);

                    uart_handle_command(command);

                    command_len = 0;
                }
            }
            else if (command_len < sizeof(command) - 1)
            {
                command[command_len++] = (char)ch;
            }
        }
    }
}

static void display_task(void *arg)
{
    sensor_data_t data;
    while (true) {
        if (xQueueReceive(display_queue, &data, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                oled_show_values(&data);
                xSemaphoreGive(i2c_mutex);
            }
            xEventGroupSetBits(cycle_events, BIT_DISPLAY_DONE);
        }
    }
}

static void flash_writer_task(void *arg)
{
    static uint8_t sector_buffer[FLASH_SECTOR_SIZE];
    memset(sector_buffer, 0xFF, sizeof(sector_buffer));

    size_t buffer_used = 0;
    size_t flash_offset = find_first_free_sector();

    if (flash_offset >= log_partition->size) {
        flash_full = true;
        logging_enabled = false;
        printf("LOG PARTITION FULL\n");
    } else {
        printf("LOG start offset: 0x%08lX\n", (unsigned long)flash_offset);
    }

    flash_message_t msg;

    while (true) {
        if (xQueueReceive(flash_queue, &msg, portMAX_DELAY) != pdTRUE) continue;

        if (msg.type == FLASH_MSG_SAMPLE) {
            if (flash_full || !logging_enabled) continue;

            if ((buffer_used + sizeof(log_record_t)) > FLASH_SECTOR_SIZE) {
                if ((flash_offset + FLASH_SECTOR_SIZE) > log_partition->size) {
                    flash_full = true;
                    logging_enabled = false;
                    printf("LOG PARTITION FULL\n");
                    continue;
                }

                esp_err_t err = write_sector(flash_offset, sector_buffer);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Flash write failed: %s", esp_err_to_name(err));
                    continue;
                }

                flash_offset += FLASH_SECTOR_SIZE;
                buffer_used = 0;
                memset(sector_buffer, 0xFF, sizeof(sector_buffer));

                if (flash_offset >= log_partition->size) {
                    flash_full = true;
                    logging_enabled = false;
                    printf("LOG PARTITION FULL\n");
                    continue;
                }
            }

            memcpy(sector_buffer + buffer_used, &msg.record, sizeof(msg.record));
            buffer_used += sizeof(msg.record);
        } else if (msg.type == FLASH_MSG_FLUSH) {
            if (buffer_used == 0) {
                printf("FLUSH buffer empty\n");
                continue;
            }

            if ((flash_offset + FLASH_SECTOR_SIZE) > log_partition->size) {
                flash_full = true;
                logging_enabled = false;
                printf("LOG PARTITION FULL\n");
                continue;
            }

            esp_err_t err = write_sector(flash_offset, sector_buffer);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Flush failed: %s", esp_err_to_name(err));
                continue;
            }

            flash_offset += FLASH_SECTOR_SIZE;
            buffer_used = 0;
            memset(sector_buffer, 0xFF, sizeof(sector_buffer));
            printf("FLUSH complete\n");

            if (flash_offset >= log_partition->size) {
                flash_full = true;
                logging_enabled = false;
                printf("LOG PARTITION FULL\n");
            }
        }
    }
}

static void manager_task(void *arg)
{
    while (true) {
        int64_t cycle_start_us = esp_timer_get_time();
        xEventGroupClearBits(cycle_events, CYCLE_DONE_BITS);
        xTaskNotifyGive(sensor_task_handle);

        EventBits_t bits = xEventGroupWaitBits(
            cycle_events,
            CYCLE_DONE_BITS,
            pdTRUE,
            pdTRUE,
            pdMS_TO_TICKS(500)
        );

        if ((bits & CYCLE_DONE_BITS) != CYCLE_DONE_BITS) ESP_LOGW(TAG, "Cycle timeout");

        int64_t elapsed_us = esp_timer_get_time() - cycle_start_us;
        int64_t sleep_us = (SAMPLE_PERIOD_MS * 1000LL) - elapsed_us;
        vTaskDelay(pdMS_TO_TICKS(150));
        
        if (sleep_us > 2000) {
            esp_sleep_enable_timer_wakeup((uint64_t)sleep_us);
            esp_light_sleep_start();
        } else {
            taskYIELD();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

static void init_i2c(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t oled_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &oled_cfg, &oled_handle));

    i2c_device_config_t bme_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bme_cfg, &bme_handle));
}

static void init_uart(void)
{
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(
        uart_driver_install(
            UART_PORT,
            256,    // RX buffer
            0,      // TX driver buffer not required
            0,
            NULL,
            0
        )
    );

    ESP_ERROR_CHECK(
        uart_param_config(
            UART_PORT,
            &cfg
        )
    );
}

static void i2c_probe_devices(void)
{
    printf("\nI2C probe:\n");

    esp_err_t err;

    err = i2c_master_probe(bus_handle, OLED_ADDR, 100);
    printf(
        "OLED 0x%02X: %s\n",
        OLED_ADDR,
        esp_err_to_name(err)
    );

    err = i2c_master_probe(bus_handle, 0x76, 100);
    printf(
        "BME  0x76: %s\n",
        esp_err_to_name(err)
    );

    err = i2c_master_probe(bus_handle, 0x77, 100);
    printf(
        "BME  0x77: %s\n",
        esp_err_to_name(err)
    );
}

void app_main(void)
{
    init_uart();

    printf("\nESP32-S3 BME280 LOGGER\n");
    printf("Commands: status | log on | log off | flush\n");

    init_i2c();
    i2c_probe_devices();

    i2c_mutex = xSemaphoreCreateMutex();
    cycle_events = xEventGroupCreate();
    uart_queue = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(sensor_data_t));
    display_queue = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(sensor_data_t));
    flash_queue = xQueueCreate(FLASH_QUEUE_LEN, sizeof(flash_message_t));

    if (!i2c_mutex || !cycle_events || !uart_queue || !display_queue || !flash_queue) {
        ESP_LOGE(TAG, "FreeRTOS allocation failed");
        return;
    }

    log_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, "logdata");
    if (!log_partition) {
        ESP_LOGE(TAG, "logdata partition not found");
        return;
    }

    ESP_LOGI(TAG, "logdata: offset=0x%08lX size=%lu",
             (unsigned long)log_partition->address,
             (unsigned long)log_partition->size);

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

    esp_err_t bme_err = bme280_init();

    if (bme_err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "BME280 init failed: %s",
            esp_err_to_name(bme_err)
        );
    } else {
        ESP_LOGI(TAG, "BME280 initialized");
    }

    oled_init();
    oled_clear();

    if (bme_err == ESP_OK) {
        oled_print(2, 0, "LOGGER START");
    } else {
        oled_print(2, 0, "BME280 ERROR");
    }

    xSemaphoreGive(i2c_mutex);
}

    xTaskCreate(sensor_task, "Sensor Task", 4096, NULL, 5, &sensor_task_handle);
    xTaskCreate(uart_task, "UART Task", 4096, NULL, 4, NULL);
    xTaskCreate(display_task, "Display Task", 4096, NULL, 3, NULL);
    xTaskCreate(flash_writer_task, "Flash Writer Task", 4096, NULL, 3, NULL);
    xTaskCreate(manager_task, "Manager Task", 4096, NULL, 6, NULL);
}
