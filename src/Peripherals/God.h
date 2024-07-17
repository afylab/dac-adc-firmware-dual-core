#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>

class God {
 private:
  DACController& dacController;
  ADCController& adcController;
  FunctionRegistry& registry;
  // Portenta_H7_Timer ITimer(TIM15);

  // bool toggle = false;

  // void TimerHandler(void) {
  // toggle = !toggle;
  // digitalWrite(LED_BUILTIN, toggle);
  // }

 public:
  God(FunctionRegistry& registry, DACController& dacController,
      ADCController& adcController)
      : registry(registry),
        dacController(dacController),
        adcController(adcController) {
    initializeRegistry();
    // pinMode(LED_BUILTIN,  OUTPUT);
    // ITimer.attachInterruptInterval(1000000, TimerHandler);
  }

  void initializeRegistry() {}

};