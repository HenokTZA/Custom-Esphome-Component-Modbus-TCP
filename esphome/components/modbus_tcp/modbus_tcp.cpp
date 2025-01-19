#include "modbus_tcp_component.h"
#include "esphome/core/log.h"

static const char *TAG = "modbus_tcp";

namespace esphome {
namespace modbus_tcp {

void ModbusTCPComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Modbus TCP...");

  // Convert the IP address string to an IPAddress object
  IPAddress ip;
  if (!ip.fromString(ip_address_.c_str())) {
    ESP_LOGE(TAG, "Invalid IP address: %s", ip_address_.c_str());
    return;
  }

  // Initialize the Modbus client with the WiFiClient, IP, port, and queue limit
  modbus_client_ = new ModbusClientTCP(client_, ip, port_, 100);  // Adjust queueLimit as needed

  // Define the callback for receiving data
  modbus_client_->onData([this](ModbusMessage response, uint32_t token) {
    ESP_LOGD(TAG, "Received Modbus response for token %u", token);

    // Check if the response is valid and corresponds to a read holding register
    if (response.size() >= 4) {
      uint8_t function_code = response.getServerFunction();
      if (function_code == READ_HOLD_REGISTER) {
        // Extract the 16-bit register value (big-endian)
        uint16_t reg_value = (static_cast<uint16_t>(response.getData(2)) << 8) | response.getData(3);
        float sensor_value = static_cast<float>(reg_value);

        // Publish the sensor value to Home Assistant
        this->publish_state(sensor_value);
      }
    }
  });

  // Define the callback for handling errors
  modbus_client_->onError([](Error error, uint32_t token) {
    ESP_LOGE(TAG, "Modbus error: %d", static_cast<int>(error));
  });

  // Attempt to connect to the Modbus TCP server
  if (!modbus_client_->connect()) {
    ESP_LOGE(TAG, "Failed to connect to %s:%u", ip_address_.c_str(), port_);
  } else {
    ESP_LOGI(TAG, "Connected to %s:%u", ip_address_.c_str(), port_);
  }
}

void ModbusTCPComponent::loop() {
  static unsigned long last_request_time = 0;
  unsigned long current_time = millis();

  // Send a request every 5000 ms (5 seconds)
  if (current_time - last_request_time >= 5000) {
    last_request_time = current_time;

    // Create a request to read a holding register
    ModbusMessage request;
    // Parameters: function code (READ_HOLD_REGISTER), slave ID (1), register address, number of registers (1)
    request.addRequest(READ_HOLD_REGISTER, 1, register_address_, 1);

    // Send the request asynchronously
    Error error = modbus_client_->addRequest(request, token_);
    if (error != SUCCESS) {
      ESP_LOGE(TAG, "Failed to send Modbus request (Error: %d)", static_cast<int>(error));
    }
  }
}

void ModbusTCPComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus TCP:");
  ESP_LOGCONFIG(TAG, "  IP Address: %s", ip_address_.c_str());
  ESP_LOGCONFIG(TAG, "  Port: %u", port_);
  ESP_LOGCONFIG(TAG, "  Register Address: 0x%X", register_address_);
}

}  // namespace modbus_tcp
}  // namespace esphome
