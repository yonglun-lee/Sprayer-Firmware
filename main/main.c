#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "rs485_comm.h"
#include <time.h>

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

void read_k24_info_task(void *pvParameters) {
  uint32_t val;
  esp_err_t err;

  char current_unit_str[8] = "";

  while (1) {
    ESP_LOGI(TAG, "--- K24 Sensor Full Information ---");

    struct {
      uint16_t addr;    // address
      uint16_t cnt;     // number of registers
      const char *name; // attribute
    } registers[] = {
        {ADDR_ADDRESS, 1, "Slave Address"},
        {ADDR_BAUDRATE, 2, "Baud Rate"},
        {ADDR_PROD_INFO, 2, "Product Info"},
        {ADDR_HW_INFO, 2, "Hardware Info"},
        {ADDR_SW_INFO, 2, "Software Info"},
        {ADDR_MEASURED_VAL, 2, "Measured Value"},
        {ADDR_SHIFT_TOTAL, 2, "Cumulative Total"},
        {ADDR_GRAND_TOTAL, 2, "Grand Total"},
        {ADDR_AVG_FLOW, 2, "Average Flow"},
        {ADDR_UNIT, 1, "Unit"},
        {ADDR_COEFFICIENT, 1, "Coefficient"},
        {ADDR_CALIB_FACT, 1, "Calibration Factor"},
        {ADDR_TIMESTAMP, 2, "Timestamp"},
        // {ADDR_DETAILS_START, 4, "Details Start"}
    };

    for (int i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
      // Pass the specific count for this register
      err = rs485_read_raw_address(registers[i].addr, registers[i].cnt, &val);

      if (err == ESP_OK) {
        if (registers[i].addr == ADDR_PROD_INFO) {
          ESP_LOGI(TAG, "[0x%04X] %-20s: K24-%04lX-%04lX", registers[i].addr,
                   registers[i].name, (val >> 16) & 0xFFFF, val & 0xFFFF);
        } else if (registers[i].addr == ADDR_HW_INFO ||
                   registers[i].addr == ADDR_SW_INFO) {
          ESP_LOGI(TAG, "[0x%04X] %-20s: K24-%04lX-V%lu.%03lu",
                   registers[i].addr, registers[i].name, (val >> 16) & 0xFFFF,
                   (val & 0xFFFF) / 1000, (val & 0xFFFF) % 1000);
        } else if (registers[i].addr == ADDR_UNIT) {
          const char *u = "Unknown";
          switch (val) {
          case 0:
            u = "Error";
            break;
          case 1:
            u = "QTS";
            break;
          case 2:
            u = "PTS";
            break;
          case 3:
            u = "Litre";
            break;
          case 4:
            u = "GAL";
            break;
          case 5:
            u = "PA";
            break; // Datasheet says "Pressure", unusual for flow meter
          case 6:
            u = "m3";
            break;
          case 7:
            u = "KG";
            break;
          default:
            u = "Unknown";
            break;
          }
          // Update current unit for next loop/display
          strncpy(current_unit_str, u, sizeof(current_unit_str) - 1);
          ESP_LOGI(TAG, "[0x%04X] %-20s: %s (%lu)", registers[i].addr,
                   registers[i].name, u, val);

        } else if (registers[i].addr == ADDR_COEFFICIENT) {
          ESP_LOGI(TAG, "[0x%04X] %-20s: %.3f", registers[i].addr,
                   registers[i].name, (float)val / 1000.0f);

        } else if (registers[i].addr == ADDR_MEASURED_VAL ||
                   registers[i].addr == ADDR_SHIFT_TOTAL ||
                   registers[i].addr == ADDR_GRAND_TOTAL) {

          // Note: We use 'current_unit_str' here, but in the very first loop
          // pass
          ESP_LOGI(TAG, "[0x%04X] %-20s: %.3f %s", registers[i].addr,
                   registers[i].name, (float)val / 1000.0f, current_unit_str);

        } else if (registers[i].addr == ADDR_AVG_FLOW) {
          // FIX: Average flow has 2 decimal places
          ESP_LOGI(TAG, "[0x%04X] %-20s: %.2f /min", registers[i].addr,
                   registers[i].name, (float)val / 100.0f);

        } else if (registers[i].addr == ADDR_TIMESTAMP) {
          time_t t = (time_t)val;
          // Adjust for Timezone (User is in +08:00 based on context)
          t += 8 * 3600;
          struct tm ts;
          localtime_r(&t, &ts);
          char buf[64];
          strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ts);
          ESP_LOGI(TAG, "[0x%04X] %-20s: %s", registers[i].addr,
                   registers[i].name, buf);

        } else {
          // Fallback for Address, Baudrate, etc.
          ESP_LOGI(TAG, "[0x%04X] %-20s: %lu (0x%08lX)", registers[i].addr,
                   registers[i].name, val, val);
        }
      } else {
        ESP_LOGW(TAG, "[0x%04X] %-20s: Read Failed (%s)", registers[i].addr,
                 registers[i].name, esp_err_to_name(err));
      }
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
  xTaskCreate(read_k24_info_task, "read_k24_info", 4096, NULL, 5, NULL);
}
