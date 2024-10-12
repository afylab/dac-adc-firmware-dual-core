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

  void beginSingleTransaction() {
    digitalWrite(led, HIGH);
    SPI.beginTransaction(spiSettings);
  }

  void endSingleTransaction() {
    SPI.endTransaction();
    digitalWrite(led, LOW);
  }

  static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }

  static void dataLedOff() { /*digitalWrite(led, LOW);*/ }

  void beginTransaction() { SPI.beginTransaction(spiSettings); }

  void endTransaction() { SPI.endTransaction(); }

  byte receiveByte() {
    return SPI.transfer(0);
  }

  void transfer(void* buf, size_t count) { SPI.transfer(buf, count); }

  uint8_t transfer(uint8_t data) { return SPI.transfer(data); }
};

bool PeripheralCommsController::spiInitialized = false;
