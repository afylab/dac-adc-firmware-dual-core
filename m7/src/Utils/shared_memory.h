#pragma once
#include <Arduino.h>

#define MESSAGE_SIZE 256
#define DEBUG_BUFFER_SIZE 1024

// Define the structures for shared data
struct SharedData {
    char message[MESSAGE_SIZE];
    volatile bool has_new_data;
};

// Initialize shared memory
bool initSharedMemory();

// M4 core functions
void m4SendData(const char* data);
bool m4CheckForNewData();
void m4GetData(char* buffer);

// M7 core functions
void m7SendData(const char* data);
bool m7CheckForNewData();
void m7GetData(char* buffer);

// Debug functions
void debugPrintMemory(const char* label, const char* data, size_t size);
const char* getDebugBuffer();
void clearDebugBuffer();