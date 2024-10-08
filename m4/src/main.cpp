#include <Arduino.h>

#include "Config.h"
#include "FunctionRegistry/FunctionRegistry.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/God.h"
#include "Peripherals/PeripheralCommsController.h"
#include <vector>
#include "UserIOHandler.h"

#include "Utils/shared_memory.h"

#include "Peripherals/God2D.h"


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  if (!initSharedMemory()) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  UserIOHandler::setup();

  PeripheralCommsController::setup();

  for (int i : dac_cs_pins) {
    DACController::addChannel(i);
  }

  for (int i : adc_cs_pins) {
    ADCController::addBoard(adc_cs_pins[i], drdy[i], reset[i]);
  }

  DACController::setup();
  ADCController::setup();

  God::setup();
  God2D::setup();
  
}



void loop() {
  UserIOHandler::handleUserIO();
  // m4SendData("Hello, World!");
  // delay(1000);
}