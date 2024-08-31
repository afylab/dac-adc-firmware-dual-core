#include "shared_memory.h"

#include <SDRAM.h>

#define SDRAM_START_ADDRESS 0x38000000

SharedMemory* shared_memory = nullptr;

bool initSharedMemory() {
  SDRAM.begin(SDRAM_START_ADDRESS);
  shared_memory = reinterpret_cast<SharedMemory*>(SDRAM_START_ADDRESS);

  shared_memory->m4_to_m7_char_buffer.read_index = 0;
  shared_memory->m4_to_m7_char_buffer.write_index = 0;
  shared_memory->m7_to_m4_char_buffer.read_index = 0;
  shared_memory->m7_to_m4_char_buffer.write_index = 0;
  shared_memory->m4_to_m7_float_buffer.read_index = 0;
  shared_memory->m4_to_m7_float_buffer.write_index = 0;
  shared_memory->m7_to_m4_float_buffer.read_index = 0;
  shared_memory->m7_to_m4_float_buffer.write_index = 0;
  shared_memory->m4_to_m7_voltage_buffer.read_index = 0;
  shared_memory->m4_to_m7_voltage_buffer.write_index = 0;
  shared_memory->m7_to_m4_voltage_buffer.read_index = 0;
  shared_memory->m7_to_m4_voltage_buffer.write_index = 0;

  return true;
}

// Char buffer operations
static bool charBufferSend(CharCircularBuffer* buffer, const char* data,
                           size_t length) {
  if (length > MAX_MESSAGE_SIZE) return false;

  uint32_t available_space =
      (buffer->read_index - buffer->write_index - 1 + CHAR_BUFFER_SIZE) %
      CHAR_BUFFER_SIZE;
  if (length + sizeof(uint32_t) > available_space) return false;

  uint32_t write_index = buffer->write_index;
  *reinterpret_cast<uint32_t*>(&buffer->buffer[write_index]) = length;
  write_index = (write_index + sizeof(uint32_t)) % CHAR_BUFFER_SIZE;

  for (size_t i = 0; i < length; ++i) {
    buffer->buffer[write_index] = data[i];
    write_index = (write_index + 1) % CHAR_BUFFER_SIZE;
  }

  buffer->write_index = write_index;
  return true;
}

static bool charBufferReceive(CharCircularBuffer* buffer, char* data,
                              size_t& length) {
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }

  uint32_t read_index = buffer->read_index;
  uint32_t msg_length =
      *reinterpret_cast<uint32_t*>(&buffer->buffer[read_index]);
  read_index = (read_index + sizeof(uint32_t)) % CHAR_BUFFER_SIZE;

  if (msg_length > length) {
    length = msg_length;
    return false;
  }

  for (size_t i = 0; i < msg_length; ++i) {
    data[i] = buffer->buffer[read_index];
    read_index = (read_index + 1) % CHAR_BUFFER_SIZE;
  }

  length = msg_length;
  buffer->read_index = read_index;
  return true;
}

static bool charBufferHasMessage(CharCircularBuffer* buffer) {
  return buffer->read_index != buffer->write_index;
}

// Float buffer operations
static bool floatBufferSend(FloatCircularBuffer* buffer, const float* data,
                            size_t length) {
  if (length > MAX_MESSAGE_SIZE) return false;

  uint32_t available_space =
      (buffer->read_index - buffer->write_index - 1 + FLOAT_BUFFER_SIZE) %
      FLOAT_BUFFER_SIZE;
  if (length + 1 > available_space) return false;

  uint32_t write_index = buffer->write_index;
  buffer->buffer[write_index] = length;
  write_index = (write_index + 1) % FLOAT_BUFFER_SIZE;

  for (size_t i = 0; i < length; ++i) {
    buffer->buffer[write_index] = data[i];
    write_index = (write_index + 1) % FLOAT_BUFFER_SIZE;
  }

  buffer->write_index = write_index;
  return true;
}

static bool floatBufferReceive(FloatCircularBuffer* buffer, float* data,
                               size_t& length) {
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }

  uint32_t read_index = buffer->read_index;
  uint32_t msg_length = static_cast<uint32_t>(buffer->buffer[read_index]);
  read_index = (read_index + 1) % FLOAT_BUFFER_SIZE;

  if (msg_length > length) {
    length = msg_length;
    return false;
  }

  for (size_t i = 0; i < msg_length; ++i) {
    data[i] = buffer->buffer[read_index];
    read_index = (read_index + 1) % FLOAT_BUFFER_SIZE;
  }

  length = msg_length;
  buffer->read_index = read_index;
  return true;
}

static bool floatBufferHasMessage(FloatCircularBuffer* buffer) {
  return buffer->read_index != buffer->write_index;
}

// Voltage buffer operations
static bool voltageBufferSend(VoltageCircularBuffer* buffer,
                              const VoltagePacket* data, size_t length) {
  if (length > MAX_MESSAGE_SIZE) return false;

  uint32_t available_space =
      (buffer->read_index - buffer->write_index - 1 + VOLTAGE_BUFFER_SIZE) %
      VOLTAGE_BUFFER_SIZE;
  if (length > available_space) return false;

  for (size_t i = 0; i < length; ++i) {
    buffer->buffer[buffer->write_index] = data[i];
    buffer->write_index = (buffer->write_index + 1) % VOLTAGE_BUFFER_SIZE;
  }

  return true;
}

static bool voltageBufferReceive(VoltageCircularBuffer* buffer,
                                 VoltagePacket* data, size_t& length) {
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }

  size_t available =
      (buffer->write_index - buffer->read_index + VOLTAGE_BUFFER_SIZE) %
      VOLTAGE_BUFFER_SIZE;
  size_t to_read = (length < available) ? length : available;

  for (size_t i = 0; i < to_read; ++i) {
    data[i] = buffer->buffer[buffer->read_index];
    buffer->read_index = (buffer->read_index + 1) % VOLTAGE_BUFFER_SIZE;
  }

  length = to_read;
  return true;
}

static bool voltageBufferHasMessage(VoltageCircularBuffer* buffer) {
  return buffer->read_index != buffer->write_index;
}

// M4 char functions
bool m4SendChar(const char* data, size_t length) {
  return charBufferSend(&shared_memory->m4_to_m7_char_buffer, data, length);
}

bool m4ReceiveChar(char* data, size_t& length) {
  return charBufferReceive(&shared_memory->m7_to_m4_char_buffer, data, length);
}

bool m4HasCharMessage() {
  return charBufferHasMessage(&shared_memory->m7_to_m4_char_buffer);
}

// M4 float functions
bool m4SendFloat(const float* data, size_t length) {
  return floatBufferSend(&shared_memory->m4_to_m7_float_buffer, data, length);
}

bool m4ReceiveFloat(float* data, size_t& length) {
  return floatBufferReceive(&shared_memory->m7_to_m4_float_buffer, data,
                            length);
}

bool m4HasFloatMessage() {
  return floatBufferHasMessage(&shared_memory->m7_to_m4_float_buffer);
}

// M4 voltage functions
bool m4SendVoltage(const VoltagePacket* data, size_t length) {
  return voltageBufferSend(&shared_memory->m4_to_m7_voltage_buffer, data,
                           length);
}

bool m4ReceiveVoltage(VoltagePacket* data, size_t& length) {
  return voltageBufferReceive(&shared_memory->m7_to_m4_voltage_buffer, data,
                              length);
}

bool m4HasVoltageMessage() {
  return voltageBufferHasMessage(&shared_memory->m7_to_m4_voltage_buffer);
}

// M7 char functions
bool m7SendChar(const char* data, size_t length) {
  return charBufferSend(&shared_memory->m7_to_m4_char_buffer, data, length);
}

bool m7ReceiveChar(char* data, size_t& length) {
  return charBufferReceive(&shared_memory->m4_to_m7_char_buffer, data, length);
}

bool m7HasCharMessage() {
  return charBufferHasMessage(&shared_memory->m4_to_m7_char_buffer);
}

// M7 float functions
bool m7SendFloat(const float* data, size_t length) {
  return floatBufferSend(&shared_memory->m7_to_m4_float_buffer, data, length);
}

bool m7ReceiveFloat(float* data, size_t& length) {
  return floatBufferReceive(&shared_memory->m4_to_m7_float_buffer, data,
                            length);
}

bool m7HasFloatMessage() {
  return floatBufferHasMessage(&shared_memory->m4_to_m7_float_buffer);
}

// M7 voltage functions
bool m7SendVoltage(const VoltagePacket* data, size_t length) {
  return voltageBufferSend(&shared_memory->m7_to_m4_voltage_buffer, data,
                           length);
}

bool m7ReceiveVoltage(VoltagePacket* data, size_t& length) {
  return voltageBufferReceive(&shared_memory->m4_to_m7_voltage_buffer, data,
                              length);
}

bool m7HasVoltageMessage() {
  return voltageBufferHasMessage(&shared_memory->m4_to_m7_voltage_buffer);
}