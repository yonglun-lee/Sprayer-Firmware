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

// Modbus Register Addresses for K24
#define ADDR_ADDRESS 0x0000
#define ADDR_BAUDRATE 0x0001
#define ADDR_PROD_INFO 0x0003
#define ADDR_HW_INFO 0x0005
#define ADDR_SW_INFO 0x0007
#define ADDR_MEASURED_VAL 0x0009
#define ADDR_SHIFT_TOTAL 0x000B
#define ADDR_GRAND_TOTAL 0x000D
#define ADDR_AVG_FLOW 0x000F
#define ADDR_UNIT 0x0012
#define ADDR_COEFFICIENT 0x0013
#define ADDR_CALIB_FACT 0x0014
#define ADDR_TIMESTAMP 0x0015
#define ADDR_DETAILS_START 0x1000

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

/**
 * @brief Read a 32-bit value (2 Modbus registers) from the K24 sensor
 *
 * @param reg_addr The starting register address
 * @param[out] val Pointer to store the 32-bit value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rs485_read_raw_address(uint16_t reg_addr, uint16_t reg_count, uint32_t *val);

#ifdef __cplusplus
}
#endif

#endif // RS485_COMM_H
