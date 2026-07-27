#include "max30102.h"
#include "esp_log.h"

static const char *TAG = "MAX30102";
static max30102_fifo_t fifo_buffer;

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
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t max30102_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t max30102_read_reg(uint8_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t max30102_read_fifo(uint32_t *red, uint32_t *ir)
{
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MAX30102_FIFO_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 5, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        *red = ((uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[2]) & 0x03FFFF;
        *ir = ((uint32_t)data[3] << 16 | (uint32_t)data[4] << 8 | (uint32_t)data[5]) & 0x03FFFF;
    }
    return ret;
}

uint8_t max30102_get_part_id(void)
{
    uint8_t id;
    max30102_read_reg(MAX30102_PART_ID, &id);
    return id;
}

void max30102_clear_fifo(void)
{
    max30102_write_reg(MAX30102_FIFO_WR_PTR, 0x00);
    max30102_write_reg(MAX30102_OVF_COUNTER, 0x00);
    max30102_write_reg(MAX30102_FIFO_RD_PTR, 0x00);
    fifo_buffer.head = 0;
    fifo_buffer.tail = 0;
    fifo_buffer.count = 0;
}

uint8_t max30102_available(void)
{
    uint8_t wr_ptr, rd_ptr;
    max30102_read_reg(MAX30102_FIFO_WR_PTR, &wr_ptr);
    max30102_read_reg(MAX30102_FIFO_RD_PTR, &rd_ptr);
    
    int8_t diff = wr_ptr - rd_ptr;
    if (diff < 0) diff += 32;
    return (uint8_t)diff;
}

esp_err_t max30102_reset(void)
{
    esp_err_t ret = max30102_write_reg(MAX30102_MODE_CONFIG, 0x40);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uint8_t status;
    uint8_t retry = 50;
    while (retry--) {
        max30102_read_reg(MAX30102_MODE_CONFIG, &status);
        if (!(status & 0x40)) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return retry > 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t max30102_shutdown(void)
{
    return max30102_write_reg(MAX30102_MODE_CONFIG, 0x80);
}

esp_err_t max30102_wakeup(void)
{
    return max30102_write_reg(MAX30102_MODE_CONFIG, 0x00);
}

float max30102_read_temperature(void)
{
    uint8_t temp_int, temp_frac;
    max30102_write_reg(MAX30102_TEMP_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uint8_t status;
    uint8_t retry = 20;
    while (retry--) {
        max30102_read_reg(MAX30102_INT_STATUS_2, &status);
        if (status & MAX30102_INT_DIE_TEMP_RDY) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    max30102_read_reg(MAX30102_TEMP_INT, &temp_int);
    max30102_read_reg(MAX30102_TEMP_FRAC, &temp_frac);
    
    return (int8_t)temp_int + temp_frac * 0.0625f;
}

esp_err_t max30102_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    
    esp_err_t ret = max30102_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reset failed");
        return ret;
    }
    
    uint8_t part_id = max30102_get_part_id();
    ESP_LOGI(TAG, "Part ID: 0x%02X", part_id);
    if (part_id != 0x15) {
        ESP_LOGW(TAG, "Unexpected part ID, expected 0x15");
    }
    
    max30102_clear_fifo();
    
    max30102_write_reg(MAX30102_FIFO_CONFIG, MAX30102_FIFO_SAMPLE_AVG_4 | 0x00);
    max30102_write_reg(MAX30102_SPO2_CONFIG, MAX30102_ADC_RANGE_4096 | MAX30102_SAMPLE_RATE_100 | MAX30102_PULSE_WIDTH_411);
    max30102_write_reg(MAX30102_LED1_PA, 0x24);
    max30102_write_reg(MAX30102_LED2_PA, 0x24);
    max30102_write_reg(MAX30102_MODE_CONFIG, MAX30102_MODE_SPO2);
    
    max30102_write_reg(MAX30102_INT_ENABLE_1, MAX30102_INT_PPG_RDY);
    
    fifo_buffer.head = 0;
    fifo_buffer.tail = 0;
    fifo_buffer.count = 0;
    
    ESP_LOGI(TAG, "MAX30102 initialized successfully");
    return ESP_OK;
}

void max30102_config_interrupt(uint8_t interrupt_enable)
{
    max30102_write_reg(MAX30102_INT_ENABLE_1, interrupt_enable);
}

void max30102_clear_interrupt(void)
{
    uint8_t status[2];
    max30102_read_reg(MAX30102_INT_STATUS_1, &status[0]);
    max30102_read_reg(MAX30102_INT_STATUS_2, &status[1]);
}

bool max30102_check_interrupt(void)
{
    return gpio_get_level(MAX30102_INT_PIN) == 0;
}
