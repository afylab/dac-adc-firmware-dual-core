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


  byte receiveByte() {
    byte b = SPI.transfer(0);
    return b;
  }


  void transfer(void* buf, size_t count) { SPI.transfer(buf, count); }

  void transfer(uint8_t data) { SPI.transfer(data); }

  void transferInTransaction(void* buf, size_t count) {
    SPI.beginTransaction(spiSettings);
    SPI.transfer(buf, count);
    SPI.endTransaction();
  }

  void transferInTransaction(uint8_t data) {
    SPI.beginTransaction(spiSettings);
    SPI.transfer(data);
    SPI.endTransaction();
  }
};

bool PeripheralCommsController::spiInitialized = false;