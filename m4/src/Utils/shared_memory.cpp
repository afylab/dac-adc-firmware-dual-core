#include "shared_memory.h"

#include <SDRAM.h>

// Define the shared memory regions in SDRAM
#define SDRAM_START_ADDRESS 0x38000000
#define M4_TO_M7_ADDRESS (SDRAM_START_ADDRESS)
#define M7_TO_M4_ADDRESS (SDRAM_START_ADDRESS + sizeof(SharedData))

volatile SharedData* m4ToM7Data = nullptr;
volatile SharedData* m7ToM4Data = nullptr;

char debugBuffer[DEBUG_BUFFER_SIZE];
int debugBufferIndex = 0;

bool initSharedMemory() {
  if (!SDRAM.begin(SDRAM_START_ADDRESS)) {
    return false;
  }

  m4ToM7Data = (SharedData*)M4_TO_M7_ADDRESS;
  m7ToM4Data = (SharedData*)M7_TO_M4_ADDRESS;

  memset((void*)m4ToM7Data, 0, sizeof(SharedData));
  memset((void*)m7ToM4Data, 0, sizeof(SharedData));

  return true;
}

void debugPrintMemory(const char* label, const char* data, size_t size) {
  int remainingSpace = DEBUG_BUFFER_SIZE - debugBufferIndex;
  int bytesToWrite =
      snprintf(debugBuffer + debugBufferIndex, remainingSpace, "%s: '", label);
  debugBufferIndex += bytesToWrite;
  remainingSpace -= bytesToWrite;

  for (size_t i = 0; i < size && i < MESSAGE_SIZE && remainingSpace > 3; i++) {
    if (data[i] == '\0') {
      bytesToWrite =
          snprintf(debugBuffer + debugBufferIndex, remainingSpace, "\\0");
    } else {
      bytesToWrite = snprintf(debugBuffer + debugBufferIndex, remainingSpace,
                              "%c", data[i]);
    }
    debugBufferIndex += bytesToWrite;
    remainingSpace -= bytesToWrite;
  }

  if (remainingSpace > 2) {
    bytesToWrite =
        snprintf(debugBuffer + debugBufferIndex, remainingSpace, "'\n");
    debugBufferIndex += bytesToWrite;
  }
}

const char* getDebugBuffer() { return debugBuffer; }

void clearDebugBuffer() {
  memset(debugBuffer, 0, DEBUG_BUFFER_SIZE);
  debugBufferIndex = 0;
}

// M4 core functions
void m4SendData(const char* data) {
  strncpy((char*)m4ToM7Data->message, data, MESSAGE_SIZE - 1);
  m4ToM7Data->message[MESSAGE_SIZE - 1] = '\0';
  __DSB();
  m4ToM7Data->has_new_data = true;
  __DSB();
}

bool m4CheckForNewData() {
  __DSB();
  return m7ToM4Data->has_new_data;
}

void m4GetData(char* buffer) {
  strncpy(buffer, (const char*)m7ToM4Data->message, MESSAGE_SIZE - 1);
  buffer[MESSAGE_SIZE - 1] = '\0';
  __DSB();
  m7ToM4Data->has_new_data = false;
  __DSB();
}

// M4 core functions for floats
void m4SendFloats(const float* data, size_t count) {
  if (count > MAX_FLOATS) count = MAX_FLOATS;
  memcpy((void*)m4ToM7Data->float_data, data, count * sizeof(float));
  __DSB();
  m4ToM7Data->num_floats = count;  // Set the number of floats
  m4ToM7Data->has_new_float_data = true;
  __DSB();
}

bool m4CheckForNewFloats() {
  __DSB();
  return m7ToM4Data->has_new_float_data;
}

size_t m4GetFloats(float* buffer) {
  size_t available_floats = m7ToM4Data->num_floats;
  if (available_floats > MAX_FLOATS) available_floats = MAX_FLOATS;
  memcpy(buffer, (const void*)m7ToM4Data->float_data,
         available_floats * sizeof(float));
  __DSB();
  m7ToM4Data->has_new_float_data = false;
  __DSB();
  return available_floats;  // Set the number of floats received
}

// M7 core functions
void m7SendData(const char* data) {
  strncpy((char*)m7ToM4Data->message, data, MESSAGE_SIZE - 1);
  m7ToM4Data->message[MESSAGE_SIZE - 1] = '\0';
  __DSB();
  m7ToM4Data->has_new_data = true;
  __DSB();
}

bool m7CheckForNewData() {
  __DSB();
  return m4ToM7Data->has_new_data;
}

void m7GetData(char* buffer) {
  strncpy(buffer, (const char*)m4ToM7Data->message, MESSAGE_SIZE - 1);
  buffer[MESSAGE_SIZE - 1] = '\0';
  __DSB();
  m4ToM7Data->has_new_data = false;
  __DSB();
}

// M7 core functions for floats
void m7SendFloats(const float* data, size_t count) {
  if (count > MAX_FLOATS) count = MAX_FLOATS;
  memcpy((void*)m7ToM4Data->float_data, data, count * sizeof(float));
  __DSB();
  m7ToM4Data->num_floats = count;  // Set the number of floats
  m7ToM4Data->has_new_float_data = true;
  __DSB();
}

bool m7CheckForNewFloats() {
  __DSB();
  return m4ToM7Data->has_new_float_data;
}

size_t m7GetFloats(float* buffer) {
  size_t available_floats = m4ToM7Data->num_floats;
  if (available_floats > MAX_FLOATS) available_floats = MAX_FLOATS;
  memcpy(buffer, (const void*)m4ToM7Data->float_data,
         available_floats * sizeof(float));
  __DSB();
  m4ToM7Data->has_new_float_data = false;
  __DSB();
  return available_floats;  // Set the number of floats received
}