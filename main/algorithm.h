#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>

#define BUFFER_SIZE 100
#define FS 100.0f

#define HR_MIN 40
#define HR_MAX 180
#define SPO2_MIN 70
#define SPO2_MAX 100

typedef struct {
    uint32_t ir_buffer[BUFFER_SIZE];
    uint32_t red_buffer[BUFFER_SIZE];
    uint16_t buffer_idx;
    bool buffer_full;
    
    float ir_dc;
    float red_dc;
    float ir_ac[BUFFER_SIZE];
    float red_ac[BUFFER_SIZE];
    
    float last_peak;
    float peaks[10];
    uint8_t peak_count;
    uint32_t sample_count;
    
    int32_t heart_rate;
    int32_t spo2;
    bool hr_valid;
    bool spo2_valid;
    
    float dc_ir_sum;
    float dc_red_sum;
    uint32_t dc_count;
} spo2_hr_data_t;

void algorithm_init(spo2_hr_data_t *data);
bool algorithm_update(spo2_hr_data_t *data, uint32_t ir, uint32_t red);
int32_t algorithm_get_heart_rate(spo2_hr_data_t *data);
int32_t algorithm_get_spo2(spo2_hr_data_t *data);
bool algorithm_hr_valid(spo2_hr_data_t *data);
bool algorithm_spo2_valid(spo2_hr_data_t *data);

#endif
