#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "max30102.h"
#include "oled.h"
#include "algorithm.h"
#include "ble.h"

static const char *TAG = "MAIN";
static spo2_hr_data_t algo_data;
static QueueHandle_t data_ready_queue = NULL;

static void IRAM_ATTR max30102_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t msg = 1;
    xQueueSendFromISR(data_ready_queue, &msg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void init_max30102_interrupt(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << MAX30102_INT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MAX30102_INT_PIN, max30102_isr_handler, NULL);
    data_ready_queue = xQueueCreate(10, sizeof(uint8_t));
    max30102_config_interrupt(MAX30102_INT_FIFO_A_FULL);
    ESP_LOGI(TAG, "MAX30102 interrupt initialized on GPIO%d", MAX30102_INT_PIN);
}

static void data_read_task(void* arg)
{
    uint8_t msg;
    int32_t last_hr = 0, last_spo2 = 0;
    bool has_hr = false, has_spo2 = false;
    uint32_t display_count = 0;
    bool int_working = true;
    uint32_t poll_count = 0;

    while (1) {
        BaseType_t qr = xQueueReceive(data_ready_queue, &msg, pdMS_TO_TICKS(100));

        if (qr == pdTRUE) {
            int_working = true;
        } else {
            poll_count++;
            if (poll_count >= 5) {
                if (int_working) {
                    ESP_LOGW(TAG, "No interrupt received, fallback to polling");
                    int_working = false;
                }
                poll_count = 0;
            }
            if (!int_working) {
                uint8_t samples_available = max30102_available();
                while (samples_available > 0) {
                    uint32_t red, ir;
                    esp_err_t ret = max30102_read_fifo(&red, &ir);
                    if (ret == ESP_OK) {
                        algorithm_update(&algo_data, ir, red);
                        if (algorithm_hr_valid(&algo_data)) {
                            last_hr = algorithm_get_heart_rate(&algo_data);
                            has_hr = true;
                        }
                        if (algorithm_spo2_valid(&algo_data)) {
                            last_spo2 = algorithm_get_spo2(&algo_data);
                            has_spo2 = true;
                        }
                    }
                    samples_available--;
                }
            } else {
                continue;
            }
        }

        uint8_t samples_available = max30102_available();
        while (samples_available > 0) {
            uint32_t red, ir;
            esp_err_t ret = max30102_read_fifo(&red, &ir);
            if (ret == ESP_OK) {
                algorithm_update(&algo_data, ir, red);
                if (algorithm_hr_valid(&algo_data)) {
                    last_hr = algorithm_get_heart_rate(&algo_data);
                    has_hr = true;
                }
                if (algorithm_spo2_valid(&algo_data)) {
                    last_spo2 = algorithm_get_spo2(&algo_data);
                    has_spo2 = true;
                }
            }
            samples_available--;
        }

        if (qr == pdTRUE) {
            max30102_clear_interrupt();
        }

        display_count++;
        if (display_count >= 10) {
            display_count = 0;
            char buf[20];

            oled_clear_line(3);
            if (has_hr) {
                snprintf(buf, sizeof(buf), "HR: %3ld BPM", (long)last_hr);
            } else {
                snprintf(buf, sizeof(buf), "HR: -- BPM");
            }
            oled_write_string(3, 0, buf);

            oled_clear_line(5);
            if (has_spo2) {
                snprintf(buf, sizeof(buf), "SpO2: %3ld %%", (long)last_spo2);
            } else {
                snprintf(buf, sizeof(buf), "SpO2: -- %%");
            }
            oled_write_string(5, 0, buf);

            oled_refresh();

            if (has_hr && has_spo2) {
                ESP_LOGI(TAG, "HR: %ld BPM, SpO2: %ld %%", (long)last_hr, (long)last_spo2);
                ble_send_health_data((uint16_t)last_hr, (uint16_t)last_spo2);
            } else {
                ESP_LOGI(TAG, "Measuring... (mode: %s)", int_working ? "interrupt" : "polling");
            }
        }
    }
}

static uint8_t i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    uint8_t found = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
            found++;
        }
    }
    
    if (found == 0) {
        ESP_LOGW(TAG, "No I2C devices found");
    } else {
        ESP_LOGI(TAG, "Found %d I2C devices", found);
    }
    return found;
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 MAX30102 SpO2/HR Monitor ===");
    
    ESP_LOGI(TAG, "Initializing I2C bus...");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    i2c_scan();
    
    ESP_LOGI(TAG, "Initializing OLED...");
    oled_init();
    oled_clear();
    oled_write_string(0, 0, "SpO2/HR Monitor");
    oled_write_string(1, 0, "Initializing...");
    oled_refresh();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Initializing MAX30102...");
    oled_write_string(0, 0, "Init MAX30102...");
    oled_refresh();
    
    esp_err_t ret = max30102_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 init failed!");
        oled_clear();
        oled_write_string(0, 0, "MAX30102 Error!");
        oled_write_string(1, 0, "Check wiring");
        oled_refresh();
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    uint8_t part_id = max30102_get_part_id();
    char id_str[16];
    snprintf(id_str, sizeof(id_str), "Part ID: 0x%02X", part_id);
    oled_clear_line(1);
    oled_write_string(1, 0, id_str);
    oled_refresh();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    algorithm_init(&algo_data);
    
    // 初始化NVS（BLE必须）
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        nvs_flash_erase();
        nvs_ret = nvs_flash_init();
    }
    ESP_LOGI(TAG, "NVS init ret=%d", nvs_ret);
    
    ble_init();
    
    // 初始化MAX30102中断
    init_max30102_interrupt();
    
    oled_clear();
    oled_write_string(0, 0, "Place finger on");
    oled_write_string(1, 0, "sensor...");
    oled_write_string(3, 0, "HR: -- BPM");
    oled_write_string(5, 0, "SpO2: -- %");
    oled_refresh();
    
    // 创建数据读取任务（中断驱动）
    xTaskCreate(data_read_task, "data_read", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Starting interrupt-driven measurement...");
    
    // 主循环（可以做其他事）
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
