#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "rs485_comm.h"


static const char *TAG = "MAIN";

void flow_sensor_task(void *pvParameters) {
  k24_sensor_data_t sensor_data;

  while (1) {
    // Read data from K24 Sensor
    esp_err_t err = rs485_read_k24_sensor(&sensor_data);

    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Flow Rate: %.2f L/min, Total Volume: %.3f L",
               sensor_data.flow_rate, sensor_data.total_volume);
    } else {
      ESP_LOGW(TAG, "Failed to read K24 sensor: %s", esp_err_to_name(err));
    }

    // Delay for 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Initializing Sprayer Firmware...");

  // Initialize RS485 Communication
  esp_err_t err = rs485_comm_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize RS485: %s", esp_err_to_name(err));
    return;
  }
  ESP_LOGI(TAG, "RS485 Initialized via Modbus RTU");

  // Create tasks
  xTaskCreate(flow_sensor_task, "flow_sensor_task", 4096, NULL, 5, NULL);
}