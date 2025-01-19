#include "modbus_tcp.h"

namespace esphome {
namespace modbus_tcp {

void ModbusTCP::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ModbusTCP (host=%s, port=%u, unit_id=%u)",
                this->host_.c_str(), this->port_, this->unit_id_);
  this->connect_();
}

void ModbusTCP::loop() {
  if (!this->client_.connected()) {
    if (this->reconnect_time_ == 0 || millis() - this->reconnect_time_ > 5000) {
      ESP_LOGW(TAG, "ModbusTCP not connected. Reconnecting...");
      this->connect_();
    }
    return;
  }

  while (this->client_.available()) {
    this->incoming_buffer_.push_back(this->client_.read());
    this->process_incoming_data_();
  }
}

void ModbusTCP::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus TCP Transport:");
  ESP_LOGCONFIG(TAG, "  Host: %s", this->host_.c_str());
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Unit ID: %u", this->unit_id_);
}

void ModbusTCP::send(modbus::ModbusFrame *frame) {
  if (!this->client_.connected()) {
    ESP_LOGW(TAG, "Not connected. Dropping frame.");
    return;
  }
  uint16_t transaction_id = this->get_new_transaction_id_();
  frame->transaction_id_ = transaction_id;

  uint16_t length = 1 + frame->get_data_length();
  std::vector<uint8_t> mbap;
  mbap.reserve(7 + frame->data_.size());

  mbap.push_back(transaction_id >> 8);
  mbap.push_back(transaction_id & 0xFF);
  mbap.push_back(0x00); // Protocol ID
  mbap.push_back(0x00);
  mbap.push_back(length >> 8);
  mbap.push_back(length & 0xFF);
  mbap.push_back(this->unit_id_);

  for (auto &b : frame->data_) {
    mbap.push_back(b);
  }

  this->client_.write(mbap.data(), mbap.size());
  this->client_.flush();
}

void ModbusTCP::connect_() {
  this->client_.stop();
  ESP_LOGI(TAG, "Connecting to %s:%u...", this->host_.c_str(), this->port_);
  if (!this->client_.connect(this->host_.c_str(), this->port_)) {
    ESP_LOGW(TAG, "Connection to %s:%u failed.", this->host_.c_str(), this->port_);
    this->reconnect_time_ = millis();
  } else {
    ESP_LOGI(TAG, "Connected!");
    this->incoming_buffer_.clear();
    this->reconnect_time_ = 0;
  }
}

void ModbusTCP::process_incoming_data_() {
  while (this->incoming_buffer_.size() >= 8) {
    uint16_t transaction_id = (this->incoming_buffer_[0] << 8) | this->incoming_buffer_[1];
    uint16_t protocol_id    = (this->incoming_buffer_[2] << 8) | this->incoming_buffer_[3];
    uint16_t length_field   = (this->incoming_buffer_[4] << 8) | this->incoming_buffer_[5];

    if (protocol_id != 0) {
      ESP_LOGW(TAG, "Invalid protocol_id=%u, discarding 1 byte...", protocol_id);
      this->incoming_buffer_.erase(this->incoming_buffer_.begin());
      continue;
    }

    size_t adu_len = 6 + length_field;
    if (this->incoming_buffer_.size() < adu_len) {
      return;
    }

    uint8_t unit_id = this->incoming_buffer_[6];
    size_t pdu_len = length_field - 1;

    std::vector<uint8_t> pdu(
      this->incoming_buffer_.begin() + 7,
      this->incoming_buffer_.begin() + 7 + pdu_len
    );

    this->incoming_buffer_.erase(
      this->incoming_buffer_.begin(),
      this->incoming_buffer_.begin() + adu_len
    );

    modbus::ModbusFrame rx_frame;
    rx_frame.transaction_id_ = transaction_id;
    rx_frame.address_ = unit_id;
    rx_frame.data_ = pdu;

    this->on_data(&rx_frame);
  }
}

void ModbusTCP::on_data(modbus::ModbusFrame *frame) {
  ESP_LOGD(TAG, "Received Modbus frame with transaction ID: %u", frame->transaction_id_);
  // Handle the Modbus frame as needed
}

uint16_t ModbusTCP::get_new_transaction_id_() {
  this->transaction_id_counter_++;
  if (this->transaction_id_counter_ == 0) {
    this->transaction_id_counter_ = 1;
  }
  return this->transaction_id_counter_;
}

}  // namespace modbus_tcp
}  // namespace esphome
