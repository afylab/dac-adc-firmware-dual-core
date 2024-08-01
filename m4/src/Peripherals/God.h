#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>

#include "Portenta_H7_TimerInterrupt.h"
// #include "RPC.h"

class God {
 private:
  static Portenta_H7_Timer adcTimer;
  static Portenta_H7_Timer dacTimer;

  static God* instance;

  inline static volatile bool adcFlag = false;
  inline static volatile bool dacFlag = false;

 public:
  static void setup() { initializeRegistry(); }

  static void initializeRegistry() {
    REGISTER_MEMBER_FUNCTION_7(bufferRampSteps, "BUFFER_RAMP_STEPS");
    REGISTER_MEMBER_FUNCTION_0(printData, "PRINT_DATA");
  }

  inline static std::vector<float> saved_data;

  static OperationResult bufferRampSteps(int adcChannel, int dacChannel,
                                         float v0, float vf, int numSteps,
                                         uint32_t adc_interval_us,
                                         uint32_t dac_interval_us) {
    saved_data.clear();
    if (adc_interval_us < 1 || dac_interval_us < 1) {
      return OperationResult::Failure("Invalid interval");
    }

    ADCController::startContinuousConversion(adcChannel);

    int steps = 0;
    int x = 0;

    const int saved_data_size = numSteps * dac_interval_us / adc_interval_us;
    float* data = new float[saved_data_size];
    bool start = false;

    float* voltSetpoints = new float[numSteps];

    for (int i = 0; i < numSteps; i++) {
      voltSetpoints[i] = v0 + (vf - v0) * i / (numSteps - 1);
    }
    
    // ulong startTimeMicros = micros();
    // ulong timeOffset = 0;

    dacTimer.attachInterruptInterval(dac_interval_us, dac_handler);
    adcTimer.attachInterruptInterval(adc_interval_us, adc_handler);

    while (x < saved_data_size) {
      if (adcFlag) {
        data[x] = ADCController::getVoltageData(adcChannel);
        x++;
        adcFlag = false;
      }
      if (dacFlag) {
        DACController::setVoltage(dacChannel, voltSetpoints[steps]);
        steps++;
        dacFlag = false;
      }
    }

    adcTimer.detachInterrupt();
    dacTimer.detachInterrupt();

    ADCController::idleMode(adcChannel);

    for (int i = 0; i < saved_data_size; i++) {
      saved_data.push_back(static_cast<float>(data[i]));
    }

    return OperationResult::Success("Done, x: " + String(x) +
                                    ", steps: " + String(steps));
  }

  static char* stringToCharBuffer(String str) {
    char* buffer = new char[str.length() + 1];
    str.toCharArray(buffer, str.length() + 1);
    return buffer;
  }

  static void adc_handler() {
    adcFlag = true;
  }

  static void dac_handler() {
    dacFlag = true;
  }

  static OperationResult printData() {
    String output = "";
    for (size_t i = 0; i < saved_data.size(); i++) {
      output += String(saved_data[i], 6) + "\n";
    }
    return OperationResult::Success(output);
  }
};

God* God::instance = nullptr;
Portenta_H7_Timer God::adcTimer = Portenta_H7_Timer(TIM14);
Portenta_H7_Timer God::dacTimer = Portenta_H7_Timer(TIM15);