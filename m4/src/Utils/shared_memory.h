#pragma once
#include <Arduino.h>

#define CHAR_BUFFER_SIZE 1024
#define FLOAT_BUFFER_SIZE 256
#define VOLTAGE_BUFFER_SIZE 512
#define MAX_MESSAGE_SIZE 256

struct CharCircularBuffer {
  char buffer[CHAR_BUFFER_SIZE];
  volatile uint32_t read_index;
  volatile uint32_t write_index;
};

struct FloatCircularBuffer {
  float buffer[FLOAT_BUFFER_SIZE];
  volatile uint32_t read_index;
  volatile uint32_t write_index;
};

struct VoltagePacket {
  uint8_t adc_id;
  uint32_t setnum;
  float voltage;
};

struct VoltageCircularBuffer {
  VoltagePacket buffer[VOLTAGE_BUFFER_SIZE];
  volatile uint32_t read_index;
  volatile uint32_t write_index;
};

struct SharedMemory {
  CharCircularBuffer m4_to_m7_char_buffer;
  CharCircularBuffer m7_to_m4_char_buffer;
  FloatCircularBuffer m4_to_m7_float_buffer;
  FloatCircularBuffer m7_to_m4_float_buffer;
  VoltageCircularBuffer m4_to_m7_voltage_buffer;
  VoltageCircularBuffer m7_to_m4_voltage_buffer;

  volatile bool stop_flag;
};

extern SharedMemory* shared_memory;

bool initSharedMemory();

// M4 char functions
bool m4SendChar(const char* data, size_t length);
bool m4ReceiveChar(char* data, size_t& length);
bool m4HasCharMessage();

// M4 float functions
bool m4SendFloat(const float* data, size_t length);
bool m4ReceiveFloat(float* data, size_t& length);
bool m4HasFloatMessage();

// M4 voltage functions
bool m4SendVoltage(const VoltagePacket* data, size_t length);
bool m4ReceiveVoltage(VoltagePacket* data, size_t& length);
bool m4HasVoltageMessage();

// M7 char functions
bool m7SendChar(const char* data, size_t length);
bool m7ReceiveChar(char* data, size_t& length);
bool m7HasCharMessage();

// M7 float functions
bool m7SendFloat(const float* data, size_t length);
bool m7ReceiveFloat(float* data, size_t& length);
bool m7HasFloatMessage();

// M7 voltage functions
bool m7SendVoltage(const VoltagePacket* data, size_t length);
bool m7ReceiveVoltage(VoltagePacket* data, size_t& length);
bool m7HasVoltageMessage();

void setStopFlag(bool value);
bool getStopFlag();