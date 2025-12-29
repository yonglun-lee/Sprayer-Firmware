#include "rs485_comm.h"
#include "hardware_pins.h"
#include <driver/uart.h>
#include <esp_log.h>
#include <stdint.h>
#include <string.h>

// Forward Declaration
static uint16_t calculate_crc16(uint8_t *buffer, uint16_t length);

static const char *RS485_TAG = "RS485_K24";

// =========================================================================
// Configuration Defines
// =========================================================================
#define UART_PORT_NUM UART_NUM_2
#define BAUD_RATE 9600
#define K24_SLAVE_ADDR 0x01
#define READ_HOLDING_REGS 0x03

// Register Addresses
#define REG_TOTAL_VOLUME_START ADDR_GRAND_TOTAL
#define REG_FLOW_RATE_START 0x0017 // Instantaneous Flow Rate

// UART Settings
#define BUF_SIZE 256
#define READ_TIMEOUT_MS 1000

// =========================================================================
// Helper Functions
// =========================================================================

// Calculate CRC16 (Modbus)
static uint16_t calculate_crc16(uint8_t *buffer, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= buffer[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Read Modbus Registers
static esp_err_t read_k24_registers(uint16_t start_reg, uint16_t reg_count, uint32_t *raw_val) {
  uint8_t cmd[8];
  cmd[0] = K24_SLAVE_ADDR;
  cmd[1] = READ_HOLDING_REGS;
  cmd[2] = (start_reg >> 8) & 0xFF;
  cmd[3] = start_reg & 0xFF;
  cmd[4] = 0x00;
  cmd[5] = reg_count; // Dynamic register count (1 or 2)

  uint16_t crc = calculate_crc16(cmd, 6);
  cmd[6] = crc & 0xFF;
  cmd[7] = (crc >> 8) & 0xFF;

  uart_flush_input(UART_PORT_NUM);

  if (uart_write_bytes(UART_PORT_NUM, (const char *)cmd, 8) != 8) {
    ESP_LOGE(RS485_TAG, "UART write failed");
    return ESP_FAIL;
  }

  // Calculate expected response length: 
  // Addr(1) + Func(1) + BytesCount(1) + Data(reg_count * 2) + CRC(2)
  int expected_len = 5 + (reg_count * 2);
  
  uint8_t data[16]; // Increased buffer size just in case
  int length = uart_read_bytes(UART_PORT_NUM, data, expected_len, pdMS_TO_TICKS(READ_TIMEOUT_MS));

  if (length < expected_len) {
    ESP_LOGW(RS485_TAG, "Rx Timeout/Short: %d (Exp: %d)", length, expected_len);
    return ESP_ERR_TIMEOUT;
  }

  // Check CRC
  uint16_t recv_crc = data[length - 2] | (data[length - 1] << 8); // CRC is always at the end
  if (calculate_crc16(data, length - 2) != recv_crc) {
    ESP_LOGE(RS485_TAG, "CRC Fail");
    return ESP_FAIL;
  }

  // Parse Data based on length
  if (reg_count == 2) {
      // 32-bit value (2 registers)
      uint32_t high_word = (data[3] << 8) | data[4];
      uint32_t low_word = (data[5] << 8) | data[6];
      *raw_val = (high_word << 16) | low_word; 
  } else {
      // 16-bit value (1 register)
      *raw_val = (data[3] << 8) | data[4];
  }

  return ESP_OK;
}

// =========================================================================
// Public API
// =========================================================================

esp_err_t rs485_comm_init(void) {
  uart_config_t uart_config = {
      .baud_rate = BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  ESP_LOGI(RS485_TAG, "Init RS485...");
  ESP_ERROR_CHECK(
      uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

  // Set Pins (Mapping from hardware_pins.h: TX->DI, RX->RO, RTS->DE)
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TTL_DI_PIN, TTL_RO_PIN,
                               TTL_DRE_PIN, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_mode(UART_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));

  return ESP_OK;
}

esp_err_t rs485_read_k24_sensor(k24_sensor_data_t *data) {
  if (!data)
    return ESP_ERR_INVALID_ARG;

  uint32_t raw_flow_int = 0;
  uint32_t raw_total_int = 0;
  esp_err_t err;

  // 1. Read Raw Values from Sensor

  // Flow Rate (Register 0x0017)
  err = read_k24_registers(REG_FLOW_RATE_START, 2, &raw_flow_int);
  if (err != ESP_OK) {
    ESP_LOGW(RS485_TAG, "Read Flow Failed: %s", esp_err_to_name(err));
    return err;
  }

  // Total Volume (Register 0x000D)
  err = read_k24_registers(REG_TOTAL_VOLUME_START, 2, &raw_total_int);
  if (err != ESP_OK) {
    ESP_LOGW(RS485_TAG, "Read Total Failed: %s", esp_err_to_name(err));
    return err;
  }

  // 2. Conversion (Scaling only, no filtering)

  float current_flow = (float)raw_flow_int / 100.0f;
  float current_total = (float)raw_total_int / 1000.0f;

  // 3. Populate Data
  data->flow_rate = current_flow;
  data->raw_flow_rate = current_flow; // Populate raw for consistency
  data->total_volume = current_total;

  ESP_LOGD(RS485_TAG, "Flow: %.2f, Total: %.3f", data->flow_rate,
           data->total_volume);

  return ESP_OK;
}

esp_err_t rs485_read_raw_address(uint16_t reg_addr, uint16_t reg_count, uint32_t *val) {
  return read_k24_registers(reg_addr, reg_count, val);
}
