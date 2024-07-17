#pragma once

#include <Arduino.h>

#include "SPI.h"

class PeripheralCommsController {
 private:
  SPISettings spiSettings;

 public:
  static bool spiInitialized;
  PeripheralCommsController(SPISettings spi_s) : spiSettings(spi_s) {}

  static void setup() {
    if (!spiInitialized) {
      SPI.begin();
      spiInitialized = true;
    }
  }

  void beginTransaction() { SPI.beginTransaction(spiSettings); }

  void endTransaction() { SPI.endTransaction(); }

  void sendByte(byte b) { SPI.transfer(b); }

  void sendBytes(const byte* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      SPI.transfer(data[i]);
    }
  }

  template <typename... byte>
  void sendBytes(byte... args) {
    // (SPI.transfer(static_cast<byte>(args)), ...);
    for (auto b : {args...}) {
      SPI.transfer(b);
    }
  }

  void sendBytesInTransaction(const byte* data, size_t len) {
    SPI.beginTransaction(spiSettings);
    for (size_t i = 0; i < len; ++i) {
      SPI.transfer(data[i]);
    }
    SPI.endTransaction();
  }

  template <typename... byte>
  void sendBytesInTransaction(byte... args) {
    SPI.beginTransaction(spiSettings);
    // (SPI.transfer(static_cast<byte>(args)), ...);
    for (auto b : {args...}) {
      SPI.transfer(b);
    }
    SPI.endTransaction();
  }

  byte receiveByte() {
    byte b = SPI.transfer(0);
    return b;
  }

  void receiveBytes(byte* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      data[i] = SPI.transfer(0);
    }
  }

  void transfer(void* buf, size_t count) { SPI.transfer(buf, count); }

  void transfer(uint8_t data) { SPI.transfer(data); }
};

bool PeripheralCommsController::spiInitialized = false;