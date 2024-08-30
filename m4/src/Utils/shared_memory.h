#pragma once
#include <Arduino.h>

#define MESSAGE_SIZE 256
#define DEBUG_BUFFER_SIZE 1024
#define MAX_FLOATS 16

// Define the structures for shared data
struct SharedData {
  char message[MESSAGE_SIZE];
  volatile bool has_new_data;
  float float_data[MAX_FLOATS];
  volatile bool has_new_float_data;
  volatile size_t num_floats;
};

// Initialize shared memory
bool initSharedMemory();

// M4 core functions
void m4SendData(const char* data);
bool m4CheckForNewData();
void m4GetData(char* buffer);

void m4SendFloats(const float* data, size_t count);
bool m4CheckForNewFloats();
size_t m4GetFloats(float* buffer);

// M7 core functions
void m7SendData(const char* data);
bool m7CheckForNewData();
void m7GetData(char* buffer);

void m7SendFloats(const float* data, size_t count);
bool m7CheckForNewFloats();
size_t m7GetFloats(float* buffer);

// Debug functions
void debugPrintMemory(const char* label, const char* data, size_t size);
const char* getDebugBuffer();
void clearDebugBuffer();