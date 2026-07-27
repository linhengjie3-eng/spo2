#ifndef BLE_H
#define BLE_H

#include <stdint.h>

void ble_init(void);
void ble_send_health_data(uint16_t heart_rate, uint16_t spo2);

#endif
