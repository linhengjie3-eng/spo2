#include "algorithm.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "ALGO";

static const float MA4_WINDOW[] = {0.25f, 0.25f, 0.25f, 0.25f};
static const uint8_t MA4_SIZE = 4;

static float apply_ma_filter(const float *kernel, uint8_t ksize, const float *signal, float *out, uint16_t len)
{
    float max_val = 0;
    for (int i = ksize - 1; i < len; i++) {
        float sum = 0;
        for (int j = 0; j < ksize; j++) {
            sum += kernel[j] * signal[i - j];
        }
        out[i] = sum;
        if (fabsf(sum) > max_val) max_val = fabsf(sum);
    }
    for (int i = 0; i < ksize - 1; i++) {
        out[i] = out[ksize - 1];
    }
    return max_val;
}

static int32_t find_peaks(const float *signal, uint16_t len, float *peaks, uint8_t max_peaks, float threshold, float min_distance)
{
    int32_t peak_count = 0;
    bool rising = false;
    float last_peak_pos = -min_distance * 2;
    float peak_val = 0;
    float peak_pos = 0;
    
    for (uint16_t i = 1; i < len - 1; i++) {
        if (signal[i] > signal[i - 1]) {
            if (!rising) {
                rising = true;
                peak_val = signal[i];
                peak_pos = i;
            } else {
                if (signal[i] > peak_val) {
                    peak_val = signal[i];
                    peak_pos = i;
                }
            }
        } else if (signal[i] < signal[i - 1] && rising) {
            rising = false;
            if (peak_val > threshold && peak_pos - last_peak_pos > min_distance) {
                if (peak_count < max_peaks) {
                    peaks[peak_count++] = peak_pos;
                    last_peak_pos = peak_pos;
                }
            }
        }
    }
    return peak_count;
}

void algorithm_init(spo2_hr_data_t *data)
{
    memset(data, 0, sizeof(spo2_hr_data_t));
    data->heart_rate = 0;
    data->spo2 = 0;
    data->hr_valid = false;
    data->spo2_valid = false;
    data->buffer_idx = 0;
    data->buffer_full = false;
    data->dc_ir_sum = 0;
    data->dc_red_sum = 0;
    data->dc_count = 0;
}

bool algorithm_update(spo2_hr_data_t *data, uint32_t ir, uint32_t red)
{
    if (ir < 1000 || red < 1000) {
        data->hr_valid = false;
        data->spo2_valid = false;
        return false;
    }
    
    data->ir_buffer[data->buffer_idx] = ir;
    data->red_buffer[data->buffer_idx] = red;
    
    data->dc_ir_sum += ir;
    data->dc_red_sum += red;
    data->dc_count++;
    
    if (data->dc_count >= 20) {
        data->ir_dc = data->dc_ir_sum / data->dc_count;
        data->red_dc = data->dc_red_sum / data->dc_count;
        data->dc_ir_sum = 0;
        data->dc_red_sum = 0;
        data->dc_count = 0;
    }
    
    data->ir_ac[data->buffer_idx] = (float)ir - data->ir_dc;
    data->red_ac[data->buffer_idx] = (float)red - data->red_dc;
    
    data->sample_count++;
    data->buffer_idx++;
    
    if (data->buffer_idx >= BUFFER_SIZE) {
        data->buffer_idx = 0;
        data->buffer_full = true;
    }
    
    if (!data->buffer_full && data->buffer_idx < BUFFER_SIZE) {
        return false;
    }
    
    uint16_t process_len = data->buffer_full ? BUFFER_SIZE : data->buffer_idx;
    
    float ir_ac_filtered[BUFFER_SIZE];
    float red_ac_filtered[BUFFER_SIZE];
    
    float ir_max = apply_ma_filter(MA4_WINDOW, MA4_SIZE, data->ir_ac, ir_ac_filtered, process_len);
    float red_max = apply_ma_filter(MA4_WINDOW, MA4_SIZE, data->red_ac, red_ac_filtered, process_len);
    
    float threshold = ir_max * 0.3f;
    float min_distance = FS * 60.0f / HR_MAX;
    
    float peaks[10];
    int32_t peak_count = find_peaks(ir_ac_filtered, process_len, peaks, 10, threshold, min_distance);
    
    if (peak_count >= 2) {
        float total_interval = 0;
        int32_t valid_intervals = 0;
        
        for (int i = 1; i < peak_count; i++) {
            float interval = peaks[i] - peaks[i - 1];
            float bpm = 60.0f * FS / interval;
            if (bpm >= HR_MIN && bpm <= HR_MAX) {
                total_interval += interval;
                valid_intervals++;
            }
        }
        
        if (valid_intervals > 0) {
            float avg_interval = total_interval / valid_intervals;
            float hr = 60.0f * FS / avg_interval;
            if (hr >= HR_MIN && hr <= HR_MAX) {
                data->heart_rate = (int32_t)(hr + 0.5f);
                data->hr_valid = true;
            } else {
                data->hr_valid = false;
            }
        }
        
        float r_sum = 0;
        int32_t r_count = 0;
        
        for (int i = 1; i < peak_count; i++) {
            int start = (int)peaks[i - 1];
            int end = (int)peaks[i];
            if (start < 0) start = 0;
            if (end >= process_len) end = process_len - 1;
            
            float ac_ir_max = 0, ac_ir_min = 1e9;
            float ac_red_max = 0, ac_red_min = 1e9;
            
            for (int j = start; j <= end; j++) {
                if (ir_ac_filtered[j] > ac_ir_max) ac_ir_max = ir_ac_filtered[j];
                if (ir_ac_filtered[j] < ac_ir_min) ac_ir_min = ir_ac_filtered[j];
                if (red_ac_filtered[j] > ac_red_max) ac_red_max = red_ac_filtered[j];
                if (red_ac_filtered[j] < ac_red_min) ac_red_min = red_ac_filtered[j];
            }
            
            float ac_ir = ac_ir_max - ac_ir_min;
            float ac_red = ac_red_max - ac_red_min;
            
            if (ac_ir > 100 && data->ir_dc > 1000 && data->red_dc > 1000) {
                float r = (ac_red / data->red_dc) / (ac_ir / data->ir_dc);
                if (r > 0.3f && r < 1.5f) {
                    r_sum += r;
                    r_count++;
                }
            }
        }
        
        if (r_count > 0) {
            float r_avg = r_sum / r_count;
            float spo2_val = 110.0f - 25.0f * r_avg;
            
            if (spo2_val >= SPO2_MIN && spo2_val <= SPO2_MAX) {
                data->spo2 = (int32_t)(spo2_val + 0.5f);
                data->spo2_valid = true;
            } else {
                data->spo2_valid = false;
            }
        } else {
            data->spo2_valid = false;
        }
    } else {
        data->hr_valid = false;
        data->spo2_valid = false;
    }
    
    return data->hr_valid || data->spo2_valid;
}

int32_t algorithm_get_heart_rate(spo2_hr_data_t *data)
{
    return data->heart_rate;
}

int32_t algorithm_get_spo2(spo2_hr_data_t *data)
{
    return data->spo2;
}

bool algorithm_hr_valid(spo2_hr_data_t *data)
{
    return data->hr_valid;
}

bool algorithm_spo2_valid(spo2_hr_data_t *data)
{
    return data->spo2_valid;
}
