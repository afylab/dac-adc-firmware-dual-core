#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>


class God {
 private:
  FunctionRegistry& registry;
  DACController& dacController;
  ADCController& adcController;


 public:
  God(FunctionRegistry& registry, DACController& dacController,
      ADCController& adcController)
      : registry(registry),
        dacController(dacController),
        adcController(adcController)
        {
  }

  void setup() {
    initializeRegistry();
  }

  void initializeRegistry() {
    REGISTER_MEMBER_FUNCTION_7(registry, bufferRampSteps, "BUFFER_RAMP_STEPS");
    REGISTER_MEMBER_FUNCTION_0(registry, printData, "PRINT_DATA");
  }

  std::vector<float> saved_data;

  OperationResult bufferRampSteps(int adcChannel, int dacChannel, float v0, float vf,
                        int numSteps, uint32_t adc_interval_us,
                        uint32_t dac_interval_us) {
    saved_data.clear();
    if (adc_interval_us < 1 || dac_interval_us < 1) {
      return OperationResult::Failure("Invalid interval");
    }

    adcController.startContinuousConversion(adcChannel);

    volatile int steps = 0;
    volatile int x = 0;

    int saved_data_size = 2 * numSteps * dac_interval_us / adc_interval_us;
    volatile float* data = new volatile float[saved_data_size];


    // dac->setVoltage(v0);
    // dac->setVoltage(v0);
    // dacController.setVoltage(dacChannel, v0);
    ulong startTimeMicros = micros();
    bool start = false;
    ulong timeOffset = 0;
    while (x < saved_data_size) {
      ulong timeMicros = micros() - startTimeMicros;
      if (start && timeMicros % adc_interval_us == 0) {
        if (x==0) {
          timeOffset = timeMicros;
        }
        data[x] = timeMicros - timeOffset;
        data[x+1] = adcController.getVoltageData(adcChannel);
        x+=2;
      }
      if (steps < numSteps && timeMicros % dac_interval_us == 0) {
        float desiredVoltage = v0 + (vf - v0) * steps / (numSteps - 1);
        dacController.setVoltage(dacChannel, desiredVoltage);
        steps++;
          start = true;
      }
      
    }

    adcController.idleMode(adcChannel);

    for (int i = 0; i < saved_data_size; i++) {
      saved_data.push_back(static_cast<float>(data[i]));
    }

    return OperationResult::Success("Done, x: " + String(x) +
                                    ", steps: " + String(steps));
  }


  OperationResult printData() {
    Serial.println("Printing data:");
    for (size_t i = 0; i < saved_data.size(); i++) {
      Serial.println(saved_data[i], 6);
    }
    return OperationResult::Success("Done");
  }


};