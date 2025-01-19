#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "ModbusClientTCP.h" // from eModbus
#include <WiFiClient.h>
#include <IPAddress.h>

namespace esphome {
namespace modbus_tcp {

class ModbusTCPComponent : public sensor::Sensor, public Component {
 public:
  void set_ip_address(const std::string &ip_address) { ip_address_ = ip_address; }
  void set_port(uint16_t port) { port_ = port; }
  void set_register_address(uint16_t register_address) { register_address_ = register_address; }

  // ESPHome component interface
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  std::string ip_address_;
  uint16_t port_{502};
  uint16_t register_address_{0x200};

  WiFiClient client_;                // WiFi client for TCP connections
  ModbusClientTCP* modbus_client_{nullptr};  // Pointer to eModbus client instance
  uint32_t token_{0};                // Token for async requests
};

}  // namespace modbus_tcp
}  // namespace esphome
