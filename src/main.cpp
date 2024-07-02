#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "FunctionRegistry.h"
#include "FunctionRegistryMacros.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/PeripheralCommsController.h"
#include "UserIOHandler.h"
#include "Peripherals/ADC/ADCController.h"

FunctionRegistry registry;
UserIOHandler userIOHandler(registry);
PeripheralCommsController dacCommsController(DAC_SPI_SETTINGS);
PeripheralCommsController adcCommsController(ADC_SPI_SETTINGS);
DACController dacController(registry, dacCommsController);
ADCController adcController(registry, adcCommsController);

void setup() {
  userIOHandler.setup();
  PeripheralCommsController::setup();

  
  for (int i : dac_cs_pins) {
    dacController.addChannel(i, ldac);
  }


  adcController.addBoard(adc_sync_pins[0], drdy[0], reset[0]);
  adcController.addBoard(adc_sync_pins[1], drdy[1], reset[1]);

  
  dacController.setup();
  adcController.setup();
}

void loop() { userIOHandler.handleUserIO(); }