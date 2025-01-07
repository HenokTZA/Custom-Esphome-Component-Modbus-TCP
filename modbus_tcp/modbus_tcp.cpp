#include "modbus_tcp.h"

namespace esphome {
namespace modbus_tcp {

static const char *TAG = "modbus_tcp";

// Utility macros to extract 16-bit big-endian values
#define MAKEWORD(high, low) (((uint16_t)(high) << 8) | (low))

// A small helper to log a vector in hex for debugging
static void log_vector_hex(const std::vector<uint8_t> &data, const char *prefix) {
  ESP_LOGD(TAG, "%s (%d bytes):", prefix, (int) data.size());
  for (size_t i = 0; i < data.size(); i++) {
    if (i % 16 == 0) ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "%02X ", data[i]);
  }
  ESP_LOGD(TAG, "");
}

void ModbusTCPComponent::setup() {
  ESP_LOGI(TAG, "Starting Modbus TCP server on port %u, unit_id %u", this->port_, this->unit_id_);

  // Initialize data arrays to zero/false
  for (size_t i = 0; i < MAX_COILS; i++) {
    this->coil_values_[i] = false;
  }
  for (size_t i = 0; i < MAX_DISCRETE_INPUTS; i++) {
    this->discrete_input_values_[i] = false;
  }
  for (size_t i = 0; i < MAX_HOLDING_REGS; i++) {
    this->holding_registers_[i] = 0;
  }
  for (size_t i = 0; i < MAX_INPUT_REGS; i++) {
    this->input_registers_[i] = 0;
  }

  // (Re)initialize the WiFiServer with the correct port
  this->server_ = WiFiServer(this->port_);
  this->server_.begin();
}

void ModbusTCPComponent::loop() {
  // Accept new incoming clients, if any
  this->accept_new_clients_();

  // Process data from existing clients
  this->process_client_data_();
}

void ModbusTCPComponent::accept_new_clients_() {
  // Check if there's a new client
  WiFiClient newClient = this->server_.available();
  if (newClient) {
    // Create a new ClientBuffer and track it
    ClientBuffer cb;
    cb.client = newClient;
    cb.buffer.clear();
    this->client_buffers_.push_back(cb);

    ESP_LOGD(TAG, "New client connected");
  }
}

void ModbusTCPComponent::process_client_data_() {
  // Iterate over each client buffer
  for (auto &cb : this->client_buffers_) {
    // Skip if not connected
    if (!cb.client.connected()) {
      continue;
    }

    // Read available data
    while (cb.client.available() > 0) {
      uint8_t byte = cb.client.read();
      cb.buffer.push_back(byte);

      // Attempt to parse packets if we have enough data
      // A minimal Modbus TCP packet is 8 bytes + function-specific data
      // Transaction ID (2), Protocol ID (2), Length (2), Unit ID (1), Function (1)
      // We'll keep trying to parse as many complete packets as we can
      this->handle_incoming_packet_(cb);
    }
  }

  // Clean up disconnected clients
  this->client_buffers_.erase(
    std::remove_if(this->client_buffers_.begin(), this->client_buffers_.end(),
      [](const ClientBuffer &cb) {
        return !cb.client.connected();
      }),
    this->client_buffers_.end()
  );
}

/**
 * Attempt to parse Modbus TCP frames from the client buffer.
 * If a complete frame is found, handle it and remove it from the buffer.
 */
void ModbusTCPComponent::handle_incoming_packet_(ClientBuffer &cb) {
  // Keep parsing as long as we have enough data for a minimal MBAP (Modbus Application Protocol) header
  while (cb.buffer.size() >= 8) {
    // The first 6 bytes are the MBAP header:
    //   - transaction_id (2 bytes)
    //   - protocol_id (2 bytes) => typically 0x0000
    //   - length (2 bytes) => number of bytes following (includes unit_id + function + data)
    uint16_t transaction_id = MAKEWORD(cb.buffer[0], cb.buffer[1]);
    uint16_t protocol_id = MAKEWORD(cb.buffer[2], cb.buffer[3]);
    uint16_t length = MAKEWORD(cb.buffer[4], cb.buffer[5]);

    // Check if we have enough data in the buffer for the entire message
    if (cb.buffer.size() < (6 + length)) {
      // Not enough data to parse the full message yet
      return;
    }

    // Extract the full frame
    // MBAP header: 6 bytes
    // Then "length" bytes (which should include 1 byte for Unit ID, 1 byte for function, etc.)
    std::vector<uint8_t> request(cb.buffer.begin(), cb.buffer.begin() + 6 + length);

    // Remove this frame from the buffer
    cb.buffer.erase(cb.buffer.begin(), cb.buffer.begin() + 6 + length);

    // Now parse the request
    // request[6] = Unit ID
    // request[7] = Function Code
    if (length < 2) {
      // Means we at least expect UnitID + Function
      // If it's not there, it's an error; but let's just ignore
      continue;
    }
    uint8_t unit_id = request[6];
    uint8_t function = request[7];

    // If the unit_id doesn't match ours, we ignore or possibly respond with an exception
    if (unit_id != this->unit_id_) {
      ESP_LOGD(TAG, "Ignoring request for unit_id %u (our unit_id is %u)", unit_id, this->unit_id_);
      continue;
    }

    // For debugging
    log_vector_hex(request, "Incoming Modbus TCP request");

    // Build a response
    std::vector<uint8_t> response;
    // MBAP header for response: echo transaction_id, protocol_id
    response.push_back(request[0]); // TID High
    response.push_back(request[1]); // TID Low
    response.push_back(request[2]); // Protocol High
    response.push_back(request[3]); // Protocol Low
    // We'll fill in the length later
    response.push_back(0x00);       // Length High (placeholder)
    response.push_back(0x00);       // Length Low  (placeholder)

    // Next: Unit ID
    response.push_back(unit_id);

    // Next: function code (may be echoed or have error bit set if exception)
    response.push_back(function);

    // Based on the function, handle accordingly
    // Note that 'request' includes the entire MBAP+PDU
    // The PDU (Protocol Data Unit) starts at request[6] for UnitID
    // So the function's data starts at request[8]
    std::vector<uint8_t> pdu_data(request.begin() + 8, request.end());

    switch (function) {
      case 0x01: // Read Coils
        this->handle_read_coils_(function, pdu_data, response);
        break;
      case 0x02: // Read Discrete Inputs
        this->handle_read_discrete_inputs_(function, pdu_data, response);
        break;
      case 0x03: // Read Holding Registers
        this->handle_read_holding_registers_(function, pdu_data, response);
        break;
      case 0x04: // Read Input Registers
        this->handle_read_input_registers_(function, pdu_data, response);
        break;
      case 0x05: // Write Single Coil
        this->handle_write_single_coil_(function, pdu_data, response);
        break;
      case 0x06: // Write Single Register
        this->handle_write_single_register_(function, pdu_data, response);
        break;
      case 0x0F: // Write Multiple Coils
        this->handle_write_multiple_coils_(function, pdu_data, response);
        break;
      case 0x10: // Write Multiple Registers
        this->handle_write_multiple_registers_(function, pdu_data, response);
        break;
      default:
        // Illegal function
        this->send_error_response_(cb.client, transaction_id, function, 0x01 /*ILLEGAL FUNCTION*/);
        continue; // skip sending 'response' normally
    }

    // Now fill in the length field in the response
    // The length = # of bytes after MBAP (excluding transaction/protocol/length itself)
    // So it’s response.size() - 6
    uint16_t resp_len = (uint16_t)(response.size() - 6);
    response[4] = (uint8_t)(resp_len >> 8);
    response[5] = (uint8_t)(resp_len & 0xFF);

    // Send the final response
    this->send_response_(cb.client, response);
  }
}

// ================================================================
//               Helper Methods for sending responses
// ================================================================

void ModbusTCPComponent::send_error_response_(WiFiClient &client, uint16_t transaction_id,
                                              uint8_t function, uint8_t error_code)
{
  std::vector<uint8_t> response;

  // Transaction ID
  response.push_back((transaction_id >> 8) & 0xFF);
  response.push_back(transaction_id & 0xFF);
  // Protocol ID (0x0000)
  response.push_back(0x00);
  response.push_back(0x00);
  // Length (3 = UnitID + Function + ErrorCode)
  response.push_back(0x00);
  response.push_back(0x03);

  // Unit ID (our unit)
  response.push_back(this->unit_id_);
  // Function code with error flag set (bit7=1)
  response.push_back(function | 0x80);
  // Exception code
  response.push_back(error_code);

  this->send_response_(client, response);
}

void ModbusTCPComponent::send_response_(WiFiClient &client, const std::vector<uint8_t> &response) {
  log_vector_hex(response, "Sending Response");
  client.write(response.data(), response.size());
}

// ================================================================
//             Function Code Handlers (skeleton implementations)
// ================================================================

/**
 * 0x01 Read Coils
 * Request format: [StartAddrHi, StartAddrLo, QuantityHi, QuantityLo]
 */
void ModbusTCPComponent::handle_read_coils_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 4) {
    // Not enough data
    response.clear();
    this->send_error_response_(response.empty() ? WiFiClient() : WiFiClient(), 0, function, 0x03);
    return;
  }

  uint16_t start_addr = MAKEWORD(request[0], request[1]);
  uint16_t quantity = MAKEWORD(request[2], request[3]);
  if ((start_addr + quantity) > MAX_COILS || quantity < 1 || quantity > 2000) {
    // Invalid address or quantity
    response.clear();
    this->send_error_response_(response.empty() ? WiFiClient() : WiFiClient(), 0, function, 0x02);
    return;
  }

  // Number of bytes in response = ceil(quantity/8)
  uint8_t byte_count = (quantity + 7) / 8;
  response.push_back(byte_count);

  // Build coil status bytes
  uint16_t current_bit = 0;
  for (uint8_t b = 0; b < byte_count; b++) {
    uint8_t val = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (current_bit < quantity) {
        bool coil = this->coil_values_[start_addr + current_bit];
        val |= (coil << i);
        current_bit++;
      }
    }
    response.push_back(val);
  }
}

/**
 * 0x02 Read Discrete Inputs
 * Format is the same as 0x01 request, but we read discrete_input_values_
 */
void ModbusTCPComponent::handle_read_discrete_inputs_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 4) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x03);
    return;
  }

  uint16_t start_addr = MAKEWORD(request[0], request[1]);
  uint16_t quantity = MAKEWORD(request[2], request[3]);
  if ((start_addr + quantity) > MAX_DISCRETE_INPUTS || quantity < 1 || quantity > 2000) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x02);
    return;
  }

  uint8_t byte_count = (quantity + 7) / 8;
  response.push_back(byte_count);

  uint16_t current_bit = 0;
  for (uint8_t b = 0; b < byte_count; b++) {
    uint8_t val = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (current_bit < quantity) {
        bool inp = this->discrete_input_values_[start_addr + current_bit];
        val |= (inp << i);
        current_bit++;
      }
    }
    response.push_back(val);
  }
}

/**
 * 0x03 Read Holding Registers
 * Request format: [StartAddrHi, StartAddrLo, QuantityHi, QuantityLo]
 */
void ModbusTCPComponent::handle_read_holding_registers_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 4) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x03);
    return;
  }

  uint16_t start_addr = MAKEWORD(request[0], request[1]);
  uint16_t quantity = MAKEWORD(request[2], request[3]);
  if ((start_addr + quantity) > MAX_HOLDING_REGS || quantity < 1 || quantity > 125) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x02);
    return;
  }

  uint8_t byte_count = quantity * 2;
  response.push_back(byte_count);

  for (uint16_t i = 0; i < quantity; i++) {
    uint16_t val = this->holding_registers_[start_addr + i];
    response.push_back((val >> 8) & 0xFF);
    response.push_back(val & 0xFF);
  }
}

/**
 * 0x04 Read Input Registers
 * Same request format as 0x03
 */
void ModbusTCPComponent::handle_read_input_registers_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 4) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x03);
    return;
  }

  uint16_t start_addr = MAKEWORD(request[0], request[1]);
  uint16_t quantity = MAKEWORD(request[2], request[3]);
  if ((start_addr + quantity) > MAX_INPUT_REGS || quantity < 1 || quantity > 125) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x02);
    return;
  }

  uint8_t byte_count = quantity * 2;
  response.push_back(byte_count);

  for (uint16_t i = 0; i < quantity; i++) {
    uint16_t val = this->input_registers_[start_addr + i];
    response.push_back((val >> 8) & 0xFF);
    response.push_back(val & 0xFF);
  }
}

/**
 * 0x05 Write Single Coil
 * Request format: [AddrHi, AddrLo, ValueHi, ValueLo]
 * Value 0xFF00 = ON, 0x0000 = OFF
 */
void ModbusTCPComponent::handle_write_single_coil_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 4) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x03);
    return;
  }

  uint16_t addr = MAKEWORD(request[0], request[1]);
  uint16_t value = MAKEWORD(request[2], request[3]);
  if (addr >= MAX_COILS) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x02);
    return;
  }

  bool new_val = (value == 0xFF00);
  this->coil_values_[addr] = new_val;

  // Echo request back
  response.insert(response.end(), request.begin(), request.begin() + 4);
}

/**
 * 0x06 Write Single Register
 * Request format: [AddrHi, AddrLo, ValueHi, ValueLo]
 */
void ModbusTCPComponent::handle_write_single_register_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 4) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x03);
    return;
  }

  uint16_t addr = MAKEWORD(request[0], request[1]);
  uint16_t value = MAKEWORD(request[2], request[3]);
  if (addr >= MAX_HOLDING_REGS) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x02);
    return;
  }

  this->holding_registers_[addr] = value;

  // Echo request back
  response.insert(response.end(), request.begin(), request.begin() + 4);
}

/**
 * 0x0F Write Multiple Coils
 * Request format:
 *   [StartAddrHi, StartAddrLo, QuantityHi, QuantityLo, ByteCount, CoilData...]
 */
void ModbusTCPComponent::handle_write_multiple_coils_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 5) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x03);
    return;
  }

  uint16_t start_addr = MAKEWORD(request[0], request[1]);
  uint16_t quantity = MAKEWORD(request[2], request[3]);
  uint8_t byte_count = request[4];
  if ((start_addr + quantity) > MAX_COILS || quantity < 1 || byte_count < ((quantity+7)/8)) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x02);
    return;
  }

  // The coil data starts at request[5], ends at request[5 + byte_count - 1]
  size_t idx = 0;
  for (uint16_t i = 0; i < quantity; i++) {
    bool val = (request[5 + (i/8)] & (1 << (i%8))) != 0;
    this->coil_values_[start_addr + i] = val;
  }

  // Response is just the start address and quantity
  response.push_back(request[0]); // StartAddrHi
  response.push_back(request[1]); // StartAddrLo
  response.push_back(request[2]); // QuantityHi
  response.push_back(request[3]); // QuantityLo
}

/**
 * 0x10 Write Multiple Registers
 * Request format:
 *   [StartAddrHi, StartAddrLo, QuantityHi, QuantityLo, ByteCount, RegData...]
 * Each register is 2 bytes
 */
void ModbusTCPComponent::handle_write_multiple_registers_(uint8_t function, const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 5) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x03);
    return;
  }

  uint16_t start_addr = MAKEWORD(request[0], request[1]);
  uint16_t quantity = MAKEWORD(request[2], request[3]);
  uint8_t byte_count = request[4];
  if ((start_addr + quantity) > MAX_HOLDING_REGS || quantity < 1 || (byte_count != quantity * 2)) {
    response.clear();
    this->send_error_response_(WiFiClient(), 0, function, 0x02);
    return;
  }

  // The register data starts at request[5]
  for (uint16_t i = 0; i < quantity; i++) {
    uint16_t hi = request[5 + (i*2)];
    uint16_t lo = request[5 + (i*2) + 1];
    this->holding_registers_[start_addr + i] = MAKEWORD(hi, lo);
  }

  // Response: echo back start address + quantity
  response.push_back(request[0]); // StartAddrHi
  response.push_back(request[1]); // StartAddrLo
  response.push_back(request[2]); // QuantityHi
  response.push_back(request[3]); // QuantityLo
}

}  // namespace modbus_tcp
}  // namespace esphome
