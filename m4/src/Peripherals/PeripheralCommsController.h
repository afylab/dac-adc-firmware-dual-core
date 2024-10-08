#pragma once

#include <Arduino.h>
#include "SPI.h"

class PeripheralCommsController {
 private:
  SPISettings spiSettings;
  SPIClass* spi;

 public:
  static bool spiInitialized;

  PeripheralCommsController(SPISettings spi_s, SPIClass* spi_p) : spiSettings(spi_s), spi(spi_p) {}

  static void setup() {
    if (!spiInitialized) {
      SPI.begin();
      SPI1.begin();
      spiInitialized = true;
    }
  }

  void beginSingleTransaction() {
    digitalWrite(led, HIGH);
    spi->beginTransaction(spiSettings);
  }

  void endSingleTransaction() {
    spi->endTransaction();
    digitalWrite(led, LOW);
  }

  static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }

  static void dataLedOff() { /*digitalWrite(led, LOW);*/ }

  void beginTransaction() { spi->beginTransaction(spiSettings); }

  void endTransaction() { spi->endTransaction(); }

  byte receiveByte() {
    return spi->transfer(0);
  }

  void transfer(void* buf, size_t count) { spi->transfer(buf, count); }

  uint8_t transfer(uint8_t data) { return spi->transfer(data); }
};

bool PeripheralCommsController::spiInitialized = false;
