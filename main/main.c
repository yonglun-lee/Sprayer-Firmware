#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "rs485_comm.h"

// static const char *TAG = "MAIN";

// void flow_sensor_task(void *pvParameters) {
//   k24_sensor_data_t sensor_data;

//   while (1) {
//     // Read data from K24 Sensor
//     esp_err_t err = rs485_read_k24_sensor(&sensor_data);

//     if (err == ESP_OK) {
//       ESP_LOGI(TAG, "Flow Rate: %.2f L/min, Total Volume: %.3f L",
//                sensor_data.flow_rate, sensor_data.total_volume);
//     } else {
//       ESP_LOGW(TAG, "Failed to read K24 sensor: %s", esp_err_to_name(err));
//     }

//     // Delay for 1 second
//     vTaskDelay(pdMS_TO_TICKS(1000));
//   }
// }

// void app_main(void) {
//   ESP_LOGI(TAG, "Initializing Sprayer Firmware...");

//   // Initialize RS485 Communication
//   esp_err_t err = rs485_comm_init();
//   if (err != ESP_OK) {
//     ESP_LOGE(TAG, "Failed to initialize RS485: %s", esp_err_to_name(err));
//     return;
//   }
//   ESP_LOGI(TAG, "RS485 Initialized via Modbus RTU");

//   // Create tasks
//   xTaskCreate(flow_sensor_task, "flow_sensor_task", 4096, NULL, 5, NULL);
// }

static const char *TAG = "MAIN";

void read_all_k24_info_task(void *pvParameters) {
  uint32_t val;
  esp_err_t err;

  while (1) {
    ESP_LOGI(TAG, "--- K24 Sensor Full Information ---");

    // List of addresses to read
    struct {
      uint16_t addr;
      const char *name;
    } registers[] = {{ADDR_ADDRESS, "Slave Address"},
                     {ADDR_BAUDRATE, "Baudrate"},
                     {ADDR_PROD_INFO, "Product Info"},
                     {ADDR_HW_INFO, "Hardware Info"},
                     {ADDR_SW_INFO, "Software Info"},
                     {ADDR_MEASURED_VAL, "Measured Value"},
                     {ADDR_SHIFT_TOTAL, "Shift Total"},
                     {ADDR_GRAND_TOTAL, "Grand Total"},
                     {ADDR_AVG_FLOW, "Average Flow"},
                     {ADDR_UNIT, "Unit"},
                     {ADDR_COEFFICIENT, "Coefficient"},
                     {ADDR_CALIB_FACT, "Calibration Factor"},
                     {ADDR_TIMESTAMP, "Timestamp"},
                     {ADDR_DETAILS_START, "Details Start"}};

    for (int i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
      err = rs485_read_raw_address(registers[i].addr, &val);
      if (err == ESP_OK) {
        ESP_LOGI(TAG, "[0x%04X] %-20s: %lu (0x%08lX)", registers[i].addr,
                 registers[i].name, val, val);
      } else {
        ESP_LOGW(TAG, "[0x%04X] %-20s: Read Failed (%s)", registers[i].addr,
                 registers[i].name, esp_err_to_name(err));
      }
      // Small delay between reads to not saturate UART
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "-----------------------------------\n");

    // Wait 5 seconds before next full read
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Starting K24 Information Reader...");

  // Initialize RS485 Communication
  esp_err_t err = rs485_comm_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize RS485: %s", esp_err_to_name(err));
    return;
  }
  ESP_LOGI(TAG, "RS485 Initialized.");

  // Create the task to read all information
  xTaskCreate(read_all_k24_info_task, "read_k24_info", 4096, NULL, 5, NULL);
}

