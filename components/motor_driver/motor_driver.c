#include "motor_driver.h"
#include <esp_check.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // For malloc if needed, though stdlib is standard
#include <driver/ledc.h>

static const char *TAG = "MD25HV";

// Internal constants for LEDC (PWM)
// LEDC (LED Control) is the ESP32 peripheral used for PWM generation.
#define MD_LEDC_TIMER LEDC_TIMER_0
#define MD_LEDC_MODE LEDC_LOW_SPEED_MODE
#define MD_LEDC_CHANNEL LEDC_CHANNEL_0
#define MD_LEDC_DUTY_RES LEDC_TIMER_13_BIT // 13-bit resolution

// Internal structure to hold instance data
struct md25hv_dev_t {
  ledc_channel_t ledc_channel;
  ledc_timer_t ledc_timer;
  ledc_mode_t ledc_mode;
  gpio_num_t dir_pin;
  ledc_timer_bit_t duty_res;
};

md25hv_handle_t md25hv_init(const md25hv_config_t *config) {
  ESP_LOGI(TAG, "Initializing MD25HV Driver...");

  if (!config) {
    ESP_LOGE(TAG, "Config is NULL");
    return NULL;
  }

  struct md25hv_dev_t *dev =
      (struct md25hv_dev_t *)calloc(1, sizeof(struct md25hv_dev_t));
  if (!dev) {
    ESP_LOGE(TAG, "Failed to allocate memory");
    return NULL;
  }

  // Use compile-time default for LEDC resources
  dev->ledc_channel = MD_LEDC_CHANNEL;
  dev->ledc_timer = MD_LEDC_TIMER;
  dev->ledc_mode = MD_LEDC_MODE;
  dev->dir_pin = config->dir_pin;
  dev->duty_res = MD_LEDC_DUTY_RES;

  // 1. Configure LEDC Timer
  ledc_timer_config_t timer_conf = {.speed_mode = MD_LEDC_MODE,
                                    .timer_num = MD_LEDC_TIMER,
                                    .duty_resolution = dev->duty_res,
                                    .freq_hz = config->pwm_freq_hz,
                                    .clk_cfg = LEDC_AUTO_CLK};
  esp_err_t err = ledc_timer_config(&timer_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "LEDC Timer Init Failed: %s", esp_err_to_name(err));
    free(dev);
    return NULL;
  }

  // 2. Configure LEDC Channel
  ledc_channel_config_t channel_conf = {.speed_mode = MD_LEDC_MODE,
                                        .channel = MD_LEDC_CHANNEL,
                                        .timer_sel = MD_LEDC_TIMER,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = config->pwm_pin,
                                        .duty = 0,
                                        .hpoint = 0};
  err = ledc_channel_config(&channel_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "LEDC Channel Init Failed: %s", esp_err_to_name(err));
    free(dev);
    return NULL;
  }

  // 3. Configure Direction GPIO
  if (config->dir_pin != GPIO_NUM_NC) {
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << config->dir_pin),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "GPIO Init Failed: %s", esp_err_to_name(err));
      free(dev);
      return NULL;
    }
    // Initialize to 0
    gpio_set_level(config->dir_pin, 0);
  }

  ESP_LOGI(TAG, "MD25HV Initialized (PWM: %d, DIR: %d)", config->pwm_pin,
           config->dir_pin);
  return dev;
}

esp_err_t md25hv_set_speed(md25hv_handle_t handle, int speed) {
  if (!handle)
    return ESP_ERR_INVALID_ARG;

  // Clamp speed
  if (speed > 100)
    speed = 100;
  if (speed < -100)
    speed = -100;

  // Determine Direction
  int dir_level = 0;
  if (speed < 0) {
    dir_level = 1;  // Reverse logic can be flipped if needed
    speed = -speed; // Make positive for duty calculation
  }

  // Set Direction Pin
  if (handle->dir_pin != GPIO_NUM_NC) {
    gpio_set_level(handle->dir_pin, dir_level);
  }

  // Calculate Duty
  // Resolution is 2^13 = 8192. Max duty is 8191.
  uint32_t duty = (8191 * speed) / 100;

  // Set PWM Duty
  esp_err_t err = ledc_set_duty(handle->ledc_mode, handle->ledc_channel, duty);
  if (err == ESP_OK) {
    err = ledc_update_duty(handle->ledc_mode, handle->ledc_channel);
  }

  return err;
}

esp_err_t md25hv_stop(md25hv_handle_t handle) {
  return md25hv_set_speed(handle, 0);
}

void md25hv_deinit(md25hv_handle_t handle) {
  if (handle) {
    ledc_stop(handle->ledc_mode, handle->ledc_channel, 0);
    free(handle);
  }
}
