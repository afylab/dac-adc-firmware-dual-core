#include <Arduino.h>

#include "Config.h"
#include "FunctionRegistry.h"
#include "FunctionRegistryMacros.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/God.h"
#include "Peripherals/PeripheralCommsController.h"
#include "RPC.h"
#include "UserIOHandler.h"

void setup() {
  UserIOHandler::setup();
  RPC.begin();

  PeripheralCommsController::setup();

  for (int i : dac_cs_pins) {
    DACController::addChannel(i, ldac);
  }

  ADCController::addBoard(adc_cs_pins[0], drdy[0], reset[0]);
  ADCController::addBoard(adc_cs_pins[1], drdy[1], reset[1]);

  DACController::setup();
  ADCController::setup();

  God::setup();
}

// static char* stringToCharBuffer(String str) {
//   char* buffer = new char[str.length() + 1];
//   str.toCharArray(buffer, str.length() + 1);
//   return buffer;
// }
// static std::vector<unsigned long> times;
void loop() {
  UserIOHandler::handleUserIO();
  // while (times.size() < 10) {
  //   times.push_back(micros());
  // }
  // for (size_t i = 1; i < times.size(); ++i) {
  //   RPC.write(stringToCharBuffer(String(times[i] - times[i - 1]) + "\n"));
  //   delay(100);
  // }
}