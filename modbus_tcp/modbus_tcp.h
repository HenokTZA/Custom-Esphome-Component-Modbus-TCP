#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

// For WiFi and networking
#ifdef USE_ESP8266
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#elif defined(USE_ESP32)
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#endif

#include <vector>

namespace esphome {
namespace modbus_tcp {

// Max sizes for demonstration. Adjust to your needs.
static const size_t MAX_COILS = 64;
static const size_t MAX_DISCRETE_INPUTS = 64;
static const size_t MAX_HOLDING_REGS = 64;
static const size_t MAX_INPUT_REGS = 64;

class ModbusTCPComponent : public Component {
 public:
  // Public methods for config
  void set_port(uint16_t port) { this->port_ = port; }
  void set_unit_id(uint8_t unit_id) { this->unit_id_ = unit_id; }

  // Standard lifecycle methods
  void setup() override;
  void loop() override;

 protected:
  // Configuration fields from YAML
  uint16_t port_{502};
  uint8_t unit_id_{1};

  // Internal server and client list
  WiFiServer server_{502};
  std::vector<WiFiClient> clients_;

  // Storage for Modbus data
  bool coil_values_[MAX_COILS];
  bool discrete_input_values_[MAX_DISCRETE_INPUTS];
  uint16_t holding_registers_[MAX_HOLDING_REGS];
  uint16_t input_registers_[MAX_INPUT_REGS];

  // Buffer for partial frame data per client (very simplistic approach)
  // Real-world usage might need a more robust ring buffer solution
  struct ClientBuffer {
    WiFiClient client;
    std::vector<uint8_t> buffer;
  };
  std::vector<ClientBuffer> client_buffers_;

  // Helper methods
  void accept_new_clients_();
  void process_client_data_();
  void handle_incoming_packet_(ClientBuffer &cb);
  void send_error_response_(WiFiClient &client, uint16_t transaction_id, uint8_t function, uint8_t error_code);
  void send_response_(WiFiClient &client, const std::vector<uint8_t> &response);

  // Utility methods to handle each function code
  void handle_read_coils_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response);
  void handle_read_discrete_inputs_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response);
  void handle_read_holding_registers_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response);
  void handle_read_input_registers_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response);
  void handle_write_single_coil_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response);
  void handle_write_single_register_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response);
  void handle_write_multiple_coils_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response);
  void handle_write_multiple_registers_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response);
};

}  // namespace modbus_tcp
}  // namespace esphome
