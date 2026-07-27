#ifndef MAX30102_H
#define MAX30102_H

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

#define MAX30102_ADDR 0x57

#define MAX30102_INT_STATUS_1 0x00
#define MAX30102_INT_STATUS_2 0x01
#define MAX30102_INT_ENABLE_1 0x02
#define MAX30102_INT_ENABLE_2 0x03
#define MAX30102_FIFO_WR_PTR 0x04
#define MAX30102_OVF_COUNTER 0x05
#define MAX30102_FIFO_RD_PTR 0x06
#define MAX30102_FIFO_DATA 0x07
#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_MODE_CONFIG 0x09
#define MAX30102_SPO2_CONFIG 0x0A
#define MAX30102_LED1_PA 0x0C
#define MAX30102_LED2_PA 0x0D
#define MAX30102_PILOT_PA 0x10
#define MAX30102_MULTI_LED_CTRL1 0x11
#define MAX30102_MULTI_LED_CTRL2 0x12
#define MAX30102_TEMP_INT 0x1F
#define MAX30102_TEMP_FRAC 0x20
#define MAX30102_TEMP_CONFIG 0x21
#define MAX30102_REV_ID 0xFE
#define MAX30102_PART_ID 0xFF

#define MAX30102_MODE_HR 0x02
#define MAX30102_MODE_SPO2 0x03
#define MAX30102_MODE_MULTI 0x07

#define MAX30102_SAMPLE_RATE_50 0x00
#define MAX30102_SAMPLE_RATE_100 0x01
#define MAX30102_SAMPLE_RATE_200 0x02
#define MAX30102_SAMPLE_RATE_400 0x03
#define MAX30102_SAMPLE_RATE_800 0x04
#define MAX30102_SAMPLE_RATE_1000 0x05
#define MAX30102_SAMPLE_RATE_1600 0x06
#define MAX30102_SAMPLE_RATE_3200 0x07

#define MAX30102_PULSE_WIDTH_69 0x00
#define MAX30102_PULSE_WIDTH_118 0x01
#define MAX30102_PULSE_WIDTH_215 0x02
#define MAX30102_PULSE_WIDTH_411 0x03

#define MAX30102_ADC_RANGE_2048 0x00
#define MAX30102_ADC_RANGE_4096 0x20
#define MAX30102_ADC_RANGE_8192 0x40
#define MAX30102_ADC_RANGE_16384 0x60

#define MAX30102_FIFO_SAMPLE_AVG_1 0x00
#define MAX30102_FIFO_SAMPLE_AVG_2 0x20
#define MAX30102_FIFO_SAMPLE_AVG_4 0x40
#define MAX30102_FIFO_SAMPLE_AVG_8 0x60
#define MAX30102_FIFO_SAMPLE_AVG_16 0x80
#define MAX30102_FIFO_SAMPLE_AVG_32 0xA0

#define MAX30102_INT_FIFO_FULL 0x80
#define MAX30102_INT_PPG_RDY 0x40
#define MAX30102_INT_ALC_OVF 0x20
#define MAX30102_INT_DIE_TEMP_RDY 0x02

#define MAX30102_INT_PIN GPIO_NUM_10
#define MAX30102_INT_FIFO_A_FULL 0x80

#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_SDA_IO 9
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define FIFO_BUFFER_SIZE 256
#define SAMPLE_COUNT 100

typedef struct {
    uint32_t red[FIFO_BUFFER_SIZE];
    uint32_t ir[FIFO_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} max30102_fifo_t;

esp_err_t max30102_init(void);
esp_err_t max30102_write_reg(uint8_t reg, uint8_t data);
esp_err_t max30102_read_reg(uint8_t reg, uint8_t *data);
esp_err_t max30102_read_fifo(uint32_t *red, uint32_t *ir);
uint8_t max30102_get_part_id(void);
void max30102_clear_fifo(void);
uint8_t max30102_available(void);
esp_err_t max30102_shutdown(void);
esp_err_t max30102_wakeup(void);
esp_err_t max30102_reset(void);
float max30102_read_temperature(void);

void max30102_config_interrupt(uint8_t interrupt_enable);
void max30102_clear_interrupt(void);
bool max30102_check_interrupt(void);

#endif
