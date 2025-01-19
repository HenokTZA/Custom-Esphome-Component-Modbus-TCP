#pragma once

#include "esphome/components/modbus/modbus.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#endif

namespace esphome {
namespace modbus_tcp {

static const char *const TAG = "modbus_tcp";

class ModbusTCP : public modbus::Modbus, public Component {
 public:
  // Setters for host, port, etc.
  void set_host(const std::string &host) { this->host_ = host; }
  void set_port(uint16_t port) { this->port_ = port; }
  void set_unit_id(uint8_t unit_id) { this->unit_id_ = unit_id; }

  // Standard lifecycle
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Called by modbus_controller to send a Modbus request
  void send(modbus::ModbusFrame *frame) override;

 protected:
  void connect_();
  void process_incoming_data_();
  uint16_t get_new_transaction_id_();
  void on_data(modbus::ModbusFrame *frame) override;  // Added for Modbus processing

  WiFiClient client_;
  std::string host_{""};
  uint16_t port_{502};
  uint8_t unit_id_{1};

  std::vector<uint8_t> incoming_buffer_;
  uint32_t reconnect_time_{0};
  uint16_t transaction_id_counter_{0};
};

}  // namespace modbus_tcp
}  // namespace esphome
