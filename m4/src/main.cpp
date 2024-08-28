#include <Arduino.h>

#include "Config.h"
#include "FunctionRegistry.h"
#include "FunctionRegistryMacros.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/God.h"
#include "Peripherals/PeripheralCommsController.h"
#include <vector>
#include "UserIOHandler.h"

void setup() {
  UserIOHandler::setup();

  PeripheralCommsController::setup();

  for (int i : dac_cs_pins) {
    DACController::addChannel(i);
  }

  ADCController::addBoard(adc_cs_pins[0], drdy[0], reset[0]);
  
  ADCController::addBoard(adc_cs_pins[1], drdy[1], reset[1]);

  DACController::setup();
  ADCController::setup();

  God::setup();
  // pinMode(24, OUTPUT);
}

void loop() {
  UserIOHandler::handleUserIO();
}