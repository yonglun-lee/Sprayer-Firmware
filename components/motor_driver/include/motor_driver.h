/**
 * @file motor_driver.h
 * @brief Driver for Cytron MD25HV DC Motor Driver
 * @author yong-lun.lee
 * @date 2025-12-29
 */

#pragma once

#include "soc/gpio_num.h"
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration structure for MD25HV driver
 */
typedef struct {
  gpio_num_t pwm_pin;          // GPIO number for PWM input
  gpio_num_t dir_pin;          // GPIO number for Direction input
  uint32_t pwm_freq_hz;  // PWM frequency (e.g. 10000 Hz)
} md25hv_config_t;

/**
 * @brief Handle for motor driver instance
 */
typedef struct md25hv_dev_t *md25hv_handle_t;

/**
 * @brief Initialize the MD25HV motor driver
 *
 * @param config Pointer to configuration structure
 * @return md25hv_handle_t Handle to the created driver instance, or NULL on
 * error
 */
md25hv_handle_t md25hv_init(const md25hv_config_t *config);

/**
 * @brief Set the motor speed and direction
 *
 * @param handle Handle to the motor driver
 * @param speed Speed percentage from -100 to 100.
 *              Positive values = Forward (DIR High/Low depending on wiring),
 *              Negative values = Reverse,
 *              0 = Stop.
 * @return esp_err_t ESP_OK on success
 */
esp_err_t md25hv_set_speed(md25hv_handle_t handle, int speed);

/**
 * @brief Stop the motor immediately (0% Duty Cycle)
 *
 * @param handle Handle to the motor driver
 * @return esp_err_t ESP_OK on success
 */
esp_err_t md25hv_stop(md25hv_handle_t handle);

/**
 * @brief Deinitialize and free resources
 *
 * @param handle Handle to the motor driver
 */
void md25hv_deinit(md25hv_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_H
