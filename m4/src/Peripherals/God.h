#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>

// #include "Portenta_H7_TimerInterrupt.h"

#include "RPC.h"

class God {
 private:
  // static Portenta_H7_Timer* adcTimer;


  static God* instance;

 public:

  static void setup() { initializeRegistry(); }

  static void initializeRegistry() {
    REGISTER_MEMBER_FUNCTION_7(bufferRampSteps, "BUFFER_RAMP_STEPS");
    REGISTER_MEMBER_FUNCTION_0(printData, "PRINT_DATA");
  }

  inline static std::vector<float> saved_data;

  static OperationResult bufferRampSteps(int adcChannel, int dacChannel, float v0,
                                  float vf, int numSteps,
                                  uint32_t adc_interval_us,
                                  uint32_t dac_interval_us) {
    saved_data.clear();
    if (adc_interval_us < 1 || dac_interval_us < 1) {
      return OperationResult::Failure("Invalid interval");
    }

    ADCController::startContinuousConversion(adcChannel);

    int steps = 0;
    int x = 0;

    int saved_data_size = 2 * numSteps * dac_interval_us / adc_interval_us;
    float* data = new float[saved_data_size];

    // if (adcTimer.attachInterruptInterval(adc_interval_us, adc_handler)) {
    //   Serial.print(F("Starting  Timer0 OK, millis() = "));
    //   Serial.println(millis());
    // } else
    //   Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
    // dac->setVoltage(v0);
    // dac->setVoltage(v0);
    DACController::setVoltage(dacChannel, v0);
    ulong startTimeMicros = micros();
    bool start = false;
    ulong timeOffset = 0;

    while (x < saved_data_size) {
      ulong timeMicros = micros() - startTimeMicros;
      if (start && timeMicros % adc_interval_us == 0) {
        if (x == 0) {
          timeOffset = timeMicros;
        }
        data[x] = timeMicros - timeOffset;
        data[x + 1] = ADCController::getVoltageData(adcChannel);
        x += 2;
      }
      if (steps < numSteps && timeMicros % dac_interval_us == 0) {
        float desiredVoltage = v0 + (vf - v0) * steps / (numSteps - 1);
        DACController::setVoltage(dacChannel, desiredVoltage);
        steps++;
        start = true;
      }
    }

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

  // static void adc_handler() { ADCController::getVoltageData(adcChannel); }

  static OperationResult printData() {
    String output = "";
    for (size_t i = 0; i < saved_data.size(); i++) {
      output += String(saved_data[i], 6) + "\n";
    }
    return OperationResult::Success(output);
  }
};

God* God::instance = nullptr;
// Portenta_H7_Timer* God::adcTimer = new Portenta_H7_Timer(TIM1);