#pragma once

#include <Arduino.h>
#include "SPI.h"
#include "Config.h"


class PeripheralCommsController1 {
 private:
  SPISettings spiSettings;

 public:
  static bool spiInitialized;

  PeripheralCommsController1(SPISettings spi_s) : spiSettings(spi_s) {}

  static void setup() {
    if (!spiInitialized) {
      SPI1.begin();
      spiInitialized = true;
    }
  }

  void beginSingleTransaction() {
    // digitalWrite(led, HIGH);
    SPI1.beginTransaction(spiSettings);
  }

  void endSingleTransaction() {
    SPI1.endTransaction();
    // digitalWrite(led, LOW);
  }

  static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }

  static void dataLedOff() { /*digitalWrite(led, LOW);*/ }

  void beginTransaction() { SPI1.beginTransaction(spiSettings); }

  void endTransaction() { SPI1.endTransaction(); }

  byte receiveByte() {
    return SPI1.transfer(0);
  }

  void transfer(void* buf, size_t count) { SPI1.transfer(buf, count); }

  uint8_t transfer(uint8_t data) { return SPI1.transfer(data); }
};

bool PeripheralCommsController1::spiInitialized = false;
