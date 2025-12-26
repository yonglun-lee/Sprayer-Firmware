/**
 * @file rs485_comm.h
 * @brief RS485 Communication for K24 Flow Sensor
 * @author yong-lun.lee
 * @date 2025-12-24
 */

#pragma once

#ifndef RS485_COMM_H
#define RS485_COMM_H

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

// Data structure for K24 Sensor Readings
typedef struct {
  float flow_rate;     // Instantaneous Flow Rate (Litre/Min)
  float total_volume;  // Cumulative Total (Litre)
  float raw_flow_rate; // Raw Flow Rate (Last Read)
} k24_sensor_data_t;

/**
 * @brief Initialize RS485 UART and GPIO
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rs485_comm_init(void);

/**
 * @brief Read data from K24 Flow Sensor via Modbus RTU
 *
 * @param[out] data Pointer to structure to store read values
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rs485_read_k24_sensor(k24_sensor_data_t *data);

#ifdef __cplusplus
}
#endif

#endif // RS485_COMM_H
