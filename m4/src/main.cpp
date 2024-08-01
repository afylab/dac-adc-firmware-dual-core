#include <Arduino.h>

#include "Config.h"
#include "FunctionRegistry.h"
#include "FunctionRegistryMacros.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/God.h"
#include "Peripherals/PeripheralCommsController.h"
#include "RPC.h"
#include <vector>
#include "UserIOHandler.h"
void setup() {
  RPC.begin();
  UserIOHandler::setup();

  PeripheralCommsController::setup();

  for (int i : dac_cs_pins) {
    DACController::addChannel(i, ldac);
  }

  ADCController::addBoard(adc_cs_pins[0], drdy[0], reset[0]);
  
  ADCController::addBoard(adc_cs_pins[1], drdy[1], reset[1]);

  DACController::setup();
  ADCController::setup();

  God::setup();
  // pinMode(24, OUTPUT);
}

// static char* stringToCharBuffer(String str) {
//   char* buffer = new char[str.length() + 1];
//   str.toCharArray(buffer, str.length() + 1);
//   return buffer;
// }

// unsigned long times[10];
int i = 0;
void loop() {
  UserIOHandler::handleUserIO();

  // for (int i = 0; i < 10; i++) {
  //   times[i] = micros();
  // }

  // for (int i = 0; i < 9; i++) {
  //   Serial.println(times[i + 1] - times[i]);
  // }
  // if (i % 10 == 0) {
  //   digitalWrite(24, HIGH);
  //   digitalWrite(24, LOW);
  // }
}