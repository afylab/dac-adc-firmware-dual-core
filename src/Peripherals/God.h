#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>
#include <Portenta_H7_TimerInterrupt.h>

#include "Portenta_H7_ISR_Timer.hpp"

class God {
 private:
  FunctionRegistry& registry;
  DACController& dacController;
  ADCController& adcController;

  Portenta_H7_Timer ITimer;

  volatile int x;

  static God* instance; // Monotheistic

 public:
  God(FunctionRegistry& registry, DACController& dacController,
      ADCController& adcController)
      : registry(registry),
        dacController(dacController),
        adcController(adcController),
        ITimer(TIM15),
        x(0) {
    instance = this;
  }

  void setup() {
    initializeRegistry();
    pinMode(LED_BUILTIN, OUTPUT);
  }

  void initializeRegistry() {
    REGISTER_MEMBER_FUNCTION_1(registry, thing, "THING");
    REGISTER_MEMBER_FUNCTION_0(registry, getX, "GETX");
  }

  OperationResult thing(float interval_ms) {
    x = 0;
    // Convert to microseconds and ensure it's within the valid range
    uint32_t interval_us = static_cast<uint32_t>(interval_ms * 1000);
    if (interval_us < 1) {
      return OperationResult::Failure("Invalid interval");
    }

    if (!ITimer.attachInterruptInterval(interval_us, God::staticTimerHandler)) {
      return OperationResult::Failure("Failed to attach timer interrupt");
    }

    return OperationResult::Success("Timer interrupt attached");
  }


  void timerHandler() {
    x += 1;
    digitalWrite(LED_BUILTIN, x % 2);
  }

  // Static wrapper for the timer handler
  static void staticTimerHandler() {
    if (instance) {
      instance->timerHandler();
    }
  }

  OperationResult getX() { return OperationResult::Success(String(x)); }

};

// Only one God allowed, we're not heretics
God* God::instance = nullptr;