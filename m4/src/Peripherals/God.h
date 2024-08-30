#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>

#include "Config.h"
#include "Utils/TimingUtil.h"

#include "Utils/shared_memory.h"

class God {
 public:
  static void setup() { initializeRegistry(); }

  static void initializeRegistry() {
    REGISTER_MEMBER_FUNCTION_VECTOR(timeSeriesBufferRampWrapper,
                                    "TIME_SERIES_BUFFER_RAMP");
    REGISTER_MEMBER_FUNCTION_VECTOR(dacLedBufferRampWrapper,
                                    "DAC_LED_BUFFER_RAMP");
    REGISTER_MEMBER_FUNCTION_0(dacChannelCalibration, "DAC_CH_CAL");
  }

  // args:
  // numDacChannels, numAdcChannels, numSteps, dacInterval_us, adcInterval_us,
  // dacchannel0, dacv00, dacvf0, dacchannel1, dacv01, dacvf1, ..., adc0, adc1,
  // ...
  static OperationResult timeSeriesBufferRampWrapper(
      const std::vector<float>& args) {
    if (args.size() < 5) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    int numDacChannels = static_cast<int>(args[0]);
    int numAdcChannels = static_cast<int>(args[1]);
    int numSteps = static_cast<int>(args[2]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[3]);
    uint32_t adc_interval_us = static_cast<uint32_t>(args[4]);

    // Check if we have enough arguments for all DAC and ADC channels
    if (args.size() !=
        static_cast<size_t>(5 + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Allocate memory for DAC and ADC channel information
    int* dacChannels = new int[numDacChannels];
    int* dacV0s = new int[numDacChannels];
    int* dacVfs = new int[numDacChannels];
    int* adcChannels = new int[numAdcChannels];

    // Parse DAC channel information
    for (int i = 0; i < numDacChannels; ++i) {
      int baseIndex = 5 + i * 3;
      dacChannels[i] = static_cast<int>(args[baseIndex]);
      dacV0s[i] = static_cast<int>(args[baseIndex + 1]);
      dacVfs[i] = static_cast<int>(args[baseIndex + 2]);
    }

    // Parse ADC channel information
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[5 + numDacChannels * 3 + i]);
    }

    return timeSeriesBufferRampBase(numDacChannels, numAdcChannels, numSteps,
                                    dac_interval_us, adc_interval_us,
                                    dacChannels, dacV0s, dacVfs, adcChannels);
  }

  static OperationResult timeSeriesBufferRampBase(
      int numDacChannels, int numAdcChannels, int numSteps,
      uint32_t dac_interval_us, uint32_t adc_interval_us, int* dacChannels,
      int* dacV0s, int* dacVfs, int* adcChannels) {
    if (adc_interval_us < 1 || dac_interval_us < 1) {
      return OperationResult::Failure("Invalid interval");
    }

    int steps = 0;
    int x = 0;

    const int saved_data_size = numSteps * dac_interval_us / adc_interval_us;

    float** dataMatrix = new float*[numAdcChannels];

    for (int i = 0; i < numAdcChannels; i++) {
      dataMatrix[i] = new float[saved_data_size];
    }

    float** voltSetpoints = new float*[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltSetpoints[i] = new float[numSteps];
      for (int j = 0; j < numSteps; j++) {
        voltSetpoints[i][j] =
            dacV0s[i] + (dacVfs[i] - dacV0s[i]) * j / (numSteps - 1);
      }
    }

    TimingUtil::setupTimersTimeSeries(dac_interval_us, adc_interval_us);

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
    }

    while (x < saved_data_size) {
      if (TimingUtil::adcFlag) {
        ADCBoard::commsController.beginTransaction();
        if (steps <= 1) {
          for (int i = 0; i < numAdcChannels; i++) {
            ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
        } else {
          for (int i = 0; i < numAdcChannels; i++) {
            dataMatrix[i][x] =
                ADCController::getVoltageDataNoTransaction(adcChannels[i]);
                char* buffer = new char[10];
                sprintf(buffer, "%f", static_cast<float>(dataMatrix[i][x]));
                m4SendData(buffer);
          }
          x++;
        }
        ADCBoard::commsController.endTransaction();
        TimingUtil::adcFlag = false;
      }
      if (TimingUtil::dacFlag && steps < numSteps + 1) {
        DACChannel::commsController.beginTransaction();
        if (steps == 0) {
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransaction(dacChannels[i],
             voltSetpoints[i][0]);
          }
        } else {
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransaction(dacChannels[i],
             voltSetpoints[i][steps-1]);
          }
        }
        DACController::toggleLdac();
        DACChannel::commsController.endTransaction();
        steps++;
        TimingUtil::dacFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
    }

    String output = "";

    for (int i = 0; i < numAdcChannels; i++) {
      output += "ADC Channel " + String(adcChannels[i]) + ":\n";
      output += String(static_cast<float>(dataMatrix[i][0]), 9);
      for (int j = 1; j < saved_data_size; j++) {
        output += "\n" + String(static_cast<float>(dataMatrix[i][j]), 9);
      }
      output += "\n";
    }
    output.remove(output.length() - 1);

    for (int i = 0; i < numAdcChannels; i++) {
      delete[] dataMatrix[i];
    }
    delete[] dataMatrix;

    for (int i = 0; i < numDacChannels; i++) {
      delete[] voltSetpoints[i];
    }
    delete[] voltSetpoints;

    return OperationResult::Success();
  }

  // args:
  // numDacChannels, numAdcChannels, numSteps, dacInterval_us,
  // dacSettlingTime_us, dacchannel0, dacv00, dacvf0, dacchannel1, dacv01,
  // dacvf1, ..., adc0, adc1,
  // ...
  static OperationResult dacLedBufferRampWrapper(
      const std::vector<float>& args) {
    if (args.size() < 5) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    int numDacChannels = static_cast<int>(args[0]);
    int numAdcChannels = static_cast<int>(args[1]);
    int numSteps = static_cast<int>(args[2]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[3]);
    uint32_t dac_settling_time_us = static_cast<uint32_t>(args[4]);

    // Check if we have enough arguments for all DAC and ADC channels
    if (args.size() !=
        static_cast<size_t>(5 + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Allocate memory for DAC and ADC channel information
    int* dacChannels = new int[numDacChannels];
    int* dacV0s = new int[numDacChannels];
    int* dacVfs = new int[numDacChannels];
    int* adcChannels = new int[numAdcChannels];

    // Parse DAC channel information
    for (int i = 0; i < numDacChannels; ++i) {
      int baseIndex = 5 + i * 3;
      dacChannels[i] = static_cast<int>(args[baseIndex]);
      dacV0s[i] = static_cast<int>(args[baseIndex + 1]);
      dacVfs[i] = static_cast<int>(args[baseIndex + 2]);
    }

    // Parse ADC channel information
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[5 + numDacChannels * 3 + i]);
    }

    return dacLedBufferRampBase(numDacChannels, numAdcChannels, numSteps,
                                dac_interval_us, dac_settling_time_us,
                                dacChannels, dacV0s, dacVfs, adcChannels);
  }

  static OperationResult dacLedBufferRampBase(int numDacChannels,
                                              int numAdcChannels, int numSteps,
                                              uint32_t dac_interval_us,
                                              uint32_t dac_settling_time_us,
                                              int* dacChannels, int* dacV0s,
                                              int* dacVfs, int* adcChannels) {
    if (dac_settling_time_us < 1 || dac_interval_us < 1 ||
        dac_settling_time_us >= dac_interval_us) {
      return OperationResult::Failure("Invalid interval or settling time");
    }

    int steps = 0;
    int x = 0;

    float** dataMatrix = new float*[numAdcChannels];

    for (int i = 0; i < numAdcChannels; i++) {
      dataMatrix[i] = new float[numSteps];
    }

    float** voltSetpoints = new float*[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltSetpoints[i] = new float[numSteps];
      for (int j = 0; j < numSteps; j++) {
        voltSetpoints[i][j] =
            dacV0s[i] + (dacVfs[i] - dacV0s[i]) * j / (numSteps - 1);
      }
    }

    // Set up timers with the same period but phase shifted
    TimingUtil::setupTimersDacLed(dac_interval_us, dac_settling_time_us);

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
    }
    while (x < numSteps) {
      if (TimingUtil::adcFlag) {
        ADCBoard::commsController.beginTransaction();
        if (steps <= 1) {
          for (int i = 0; i < numAdcChannels; i++) {
            ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
        } else {
          for (int i = 0; i < numAdcChannels; i++) {
            dataMatrix[i][x] =
                ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
          x++;
        }
        ADCBoard::commsController.endTransaction();
        TimingUtil::adcFlag = false;
      }
      if (TimingUtil::dacFlag && steps < numSteps + 1) {
        DACChannel::commsController.beginTransaction();
        if (steps == 0) {
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransaction(dacChannels[i],
                                                   voltSetpoints[i][0]);
          }
        } else {
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransaction(dacChannels[i],
                                                   voltSetpoints[i][steps - 1]);
          }
        }
        DACController::toggleLdac();
        DACChannel::commsController.endTransaction();
        steps++;
        TimingUtil::dacFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
    }

    String output = "";

    for (int i = 0; i < numAdcChannels; i++) {
      output += "ADC Channel " + String(adcChannels[i]) + ":\n";
      output += String(static_cast<float>(dataMatrix[i][0]), 9);
      for (int j = 1; j < numSteps; j++) {
        output += "\n" + String(static_cast<float>(dataMatrix[i][j]), 9);
      }
      output += "\n";
    }
    output.remove(output.length() - 1);

    for (int i = 0; i < numAdcChannels; i++) {
      delete[] dataMatrix[i];
    }
    delete[] dataMatrix;

    for (int i = 0; i < numDacChannels; i++) {
      delete[] voltSetpoints[i];
    }
    delete[] voltSetpoints;

    return OperationResult::Success(output);
  }

  static char* stringToCharBuffer(String str) {
    char* buffer = new char[str.length() + 1];
    str.toCharArray(buffer, str.length() + 1);
    return buffer;
  }

  static OperationResult dacChannelCalibration() {
    for (int i = 0; i < NUM_CHANNELS_PER_DAC_BOARD; i++) {
      DACController::initialize();
      DACController::setCalibration(i, 0, 1);
      DACController::setVoltage(i, 0);
      delay(1);
      float offsetError = ADCController::getVoltage(i);
      DACController::setCalibration(i, offsetError, 1);
      float voltSet = 9.0;
      DACController::setVoltage(i, voltSet);
      delay(1);
      float gainError = (ADCController::getVoltage(i) - offsetError) / voltSet;
      DACController::setCalibration(i, offsetError, gainError);
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }
};
