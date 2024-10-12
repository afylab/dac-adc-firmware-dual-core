#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>

#include "Config.h"
#include "Utils/TimingUtil.h"
#include "Utils/shared_memory.h"
#include "unordered_set"

class God {
 public:
  static void setup() { initializeRegistry(); }

  static void initializeRegistry() {
    registerMemberFunctionVector(timeSeriesBufferRampWrapper,
                                 "TIME_SERIES_BUFFER_RAMP");
    registerMemberFunctionVector(dacLedBufferRampWrapper,
                                 "DAC_LED_BUFFER_RAMP");
    registerMemberFunction(dacChannelCalibration, "DAC_CH_CAL");
    registerMemberFunctionVector(boxcarAverageRamp,
                                 "BOXCAR_BUFFER_RAMP");
  }

  // args:
  // numDacChannels, numAdcChannels, numSteps, dacInterval_us, adcInterval_us,
  // dacchannel0, dacv00, dacvf0, dacchannel1, dacv01, dacvf1, ..., adc0, adc1,
  // adc2, ...
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

    if (args.size() !=
        static_cast<size_t>(5 + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    int* dacChannels = new int[numDacChannels];
    float* dacV0s = new float[numDacChannels];
    float* dacVfs = new float[numDacChannels];
    int* adcChannels = new int[numAdcChannels];

    for (int i = 0; i < numDacChannels; ++i) {
      int baseIndex = 5 + i * 3;
      dacChannels[i] = static_cast<int>(args[baseIndex]);
      dacV0s[i] = static_cast<float>(args[baseIndex + 1]);
      dacVfs[i] = static_cast<float>(args[baseIndex + 2]);
    }

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
      float* dacV0s, float* dacVfs, int* adcChannels) {
    if (adc_interval_us < 1 || dac_interval_us < 1) {
      return OperationResult::Failure("Invalid interval");
    }
    if (numSteps < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }
    if (numDacChannels < 1 || numAdcChannels < 1) {
      return OperationResult::Failure("Invalid number of channels");
    }

    int steps = 0;
    int x = 0;

    const int saved_data_size = numSteps * dac_interval_us / adc_interval_us;

    float* voltageStepSize = new float[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSize[i] = (dacVfs[i] - dacV0s[i]) / (numSteps - 1);
    }

    float* previousVoltageSet = new float[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      previousVoltageSet[i] = dacV0s[i];
    }

    TimingUtil::setupTimersTimeSeries(dac_interval_us, adc_interval_us);

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
    }

    while (x < saved_data_size && !getStopFlag()) {
      if (TimingUtil::adcFlag) {
        if (steps <= 1) {
          for (int i = 0; i < numAdcChannels; i++) {
            ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
        } else {
          float* packets = new float[numAdcChannels];
          for (int i = 0; i < numAdcChannels; i++) {
            float v =
                ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            packets[i] = v;
          }
          m4SendVoltage(packets, numAdcChannels);
          delete[] packets;
          x++;
        }
        TimingUtil::adcFlag = false;
      }
      if (TimingUtil::dacFlag && steps < numSteps + 1) {
        if (steps == 0) {
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                         dacV0s[i]);
          }
        } else {
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                         previousVoltageSet[i]);
            previousVoltageSet[i] += voltageStepSize[i];
          }
        }
        DACController::toggleLdac();
        steps++;
        TimingUtil::dacFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
    }

    PeripheralCommsController::dataLedOff();

    delete[] voltageStepSize;
    delete[] previousVoltageSet;

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }

  // args:
  // numDacChannels, numAdcChannels, numSteps, numAdcAverages, dacInterval_us,
  // dacSettlingTime_us, dacchannel0, dacv00, dacvf0, dacchannel1, dacv01,
  // dacvf1, ..., adc0, adc1, adc2, ...
  static OperationResult dacLedBufferRampWrapper(
      const std::vector<float>& args) {
    if (args.size() < 10) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    int numDacChannels = static_cast<int>(args[0]);
    int numAdcChannels = static_cast<int>(args[1]);
    int numSteps = static_cast<int>(args[2]);
    int numAdcAverages = static_cast<int>(args[3]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[4]);
    uint32_t dac_settling_time_us = static_cast<uint32_t>(args[5]);

    if (args.size() !=
        static_cast<size_t>(6 + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    int* dacChannels = new int[numDacChannels];
    float* dacV0s = new float[numDacChannels];
    float* dacVfs = new float[numDacChannels];
    int* adcChannels = new int[numAdcChannels];

    for (int i = 0; i < numDacChannels; ++i) {
      int baseIndex = 6 + i * 3;
      dacChannels[i] = static_cast<int>(args[baseIndex]);
      dacV0s[i] = static_cast<float>(args[baseIndex + 1]);
      dacVfs[i] = static_cast<float>(args[baseIndex + 2]);
    }

    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[6 + numDacChannels * 3 + i]);
    }

    return dacLedBufferRampBase(numDacChannels, numAdcChannels, numSteps,
                                numAdcAverages, dac_interval_us,
                                dac_settling_time_us, dacChannels, dacV0s,
                                dacVfs, adcChannels);
  }

  static OperationResult dacLedBufferRampBase(
      int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
      uint32_t dac_interval_us, uint32_t dac_settling_time_us, int* dacChannels,
      float* dacV0s, float* dacVfs, int* adcChannels) {
    if (dac_settling_time_us < 1 || dac_interval_us < 1 ||
        dac_settling_time_us >= dac_interval_us) {
      return OperationResult::Failure("Invalid interval or settling time");
    }
    if (numAdcAverages < 1) {
      return OperationResult::Failure("Invalid number of ADC averages");
    }
    if (numSteps < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }
    if (numDacChannels < 1 || numAdcChannels < 1) {
      return OperationResult::Failure("Invalid number of channels");
    }
    for (int i = 0; i < numAdcChannels; i++) {
      if (dac_settling_time_us <
          ADCController::getConversionTimeFloat(adcChannels[i])) {
        return OperationResult::Failure(
            "DAC settling time too short for ADC conversion time");
      }
    }

    int steps = 0;
    int x = 0;

    float* voltageStepSize = new float[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSize[i] = (dacVfs[i] - dacV0s[i]) / (numSteps - 1);
    }

    float* previousVoltageSet = new float[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      previousVoltageSet[i] = dacV0s[i];
    }

    float numAdcAveragesInv = 1.0 / static_cast<float>(numAdcAverages);

    // Set up timers with the same period but phase shifted
    TimingUtil::setupTimersDacLed(dac_interval_us, dac_settling_time_us);

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
    }
    while (x < numSteps && !getStopFlag()) {
      if (TimingUtil::adcFlag) {
        if (steps <= 1) {
          for (int i = 0; i < numAdcChannels; i++) {
            for (int j = 0; j < numAdcAverages; j++) {
              ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            }
          }
        } else {
          float* packets = new float[numAdcChannels];
          for (int i = 0; i < numAdcChannels; i++) {
            float total = 0.0;
            for (int j = 0; j < numAdcAverages; j++) {
              total +=
                  ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            }
            float v = total * numAdcAveragesInv;
            packets[i] = v;
          }
          m4SendFloat(packets, numAdcChannels);
          delete[] packets;
          x++;
        }
        TimingUtil::adcFlag = false;
      }
      if (TimingUtil::dacFlag && steps < numSteps + 1) {
        if (steps == 0) {
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                         dacV0s[i]);
          }
        } else {
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                          previousVoltageSet[i]);
            previousVoltageSet[i] += voltageStepSize[i];
          }
        }
        DACController::toggleLdac();
        steps++;
        TimingUtil::dacFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
    }

    PeripheralCommsController::dataLedOff();

    delete[] voltageStepSize;
    delete[] previousVoltageSet;

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success();
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

  // args: numDacChannels, numAdcChannels, numDacSteps,
  // numAdcMeasuresPerDacStep, numAdcAverages, numAdcConversionSkips,
  // adcConversionTime_us, {for each dac channel: dac channel, v0_1, vf_1, v0_2,
  // vf_2}, {for each adc channel: adc channel}
  static OperationResult boxcarAverageRamp(
      const std::vector<float>& args) {
    size_t currentIndex = 0;

    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numDacSteps = static_cast<int>(args[currentIndex++]);
    int numAdcMeasuresPerDacStep = static_cast<int>(args[currentIndex++]);
    int numAdcAverages = static_cast<int>(args[currentIndex++]);
    int numAdcConversionSkips = static_cast<int>(args[currentIndex++]);
    uint32_t adcConversionTime_us = static_cast<uint32_t>(args[currentIndex++]);

    int* dacChannels = new int[numDacChannels];
    float* dacV0_1 = new float[numDacChannels];
    float* dacVf_1 = new float[numDacChannels];
    float* dacV0_2 = new float[numDacChannels];
    float* dacVf_2 = new float[numDacChannels];

    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[currentIndex++]);
      dacV0_1[i] = args[currentIndex++];
      dacVf_1[i] = args[currentIndex++];
      dacV0_2[i] = args[currentIndex++];
      dacVf_2[i] = args[currentIndex++];
    }

    int* adcChannels = new int[numAdcChannels];

    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    uint32_t actualConversionTime_us = ADCController::presetConversionTime(
        adcChannels[0], adcConversionTime_us, numAdcChannels > 1);
    for (int i = 1; i < numAdcChannels; ++i) {
      ADCController::presetConversionTime(adcChannels[i], adcConversionTime_us,
                                          numAdcChannels > 1);
    }

    uint32_t dacPeriod_us =
        (numAdcMeasuresPerDacStep + numAdcConversionSkips) * (actualConversionTime_us + 5) * numAdcChannels;

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    float* voltageStepSizeLow = new float[numDacChannels];
    float* voltageStepSizeHigh = new float[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSizeLow[i] =
          (dacVf_1[i] - dacV0_1[i]) / static_cast<float>(numDacSteps - 1);
      voltageStepSizeHigh[i] =
          (dacVf_2[i] - dacV0_2[i]) / static_cast<float>(numDacSteps - 1);
    }

    float* previousVoltageSetLow = new float[numDacChannels];
    float* previousVoltageSetHigh = new float[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      previousVoltageSetLow[i] = dacV0_1[i];
      previousVoltageSetHigh[i] = dacV0_2[i];
    }

    int steps = 0;
    int totalSteps = 2 * numDacSteps * numAdcAverages + 1;
    int x = 0;
    int total_data_size = (totalSteps-1) * numAdcMeasuresPerDacStep;
    int adcGetsSinceLastDacSet = 0;

    for (int i = 0; i < numAdcChannels; ++i) {
      ADCController::startContinuousConversion(adcChannels[i]);
    }

    TimingUtil::setupTimersTimeSeries(dacPeriod_us, actualConversionTime_us);

    while (x < total_data_size && !getStopFlag()) {
      if (TimingUtil::adcFlag && x < (steps-1) * numAdcMeasuresPerDacStep) {
        if (steps <= 1) {
          for (int i = 0; i < numAdcChannels; i++) {
            ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
        } else {
          if (adcGetsSinceLastDacSet >= numAdcConversionSkips) {
            float* packets = new float[numAdcChannels];
            for (int i = 0; i < numAdcChannels; i++) {
              float v =
                  ADCController::getVoltageDataNoTransaction(adcChannels[i]);
              packets[i] = v;
            }
            m4SendVoltage(packets, numAdcChannels);
            delete[] packets;
            x++;
          }
        }
        adcGetsSinceLastDacSet++;
        TimingUtil::adcFlag = false;
      }
      if (TimingUtil::dacFlag && steps < totalSteps) {
        if (steps == 0) {
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                         dacV0_1[i]);
          }
        } else {
          for (int i = 0; i < numDacChannels; i++) {
            float currentVoltage;
            if (steps % (2 * numAdcAverages) != 0) {
              if (steps % 2 == 0) {
                currentVoltage = previousVoltageSetLow[i];
              } else {
                currentVoltage = previousVoltageSetHigh[i];
              }
            } else if (steps % 2 == 0) {
              previousVoltageSetLow[i] += voltageStepSizeLow[i];
              previousVoltageSetHigh[i] += voltageStepSizeHigh[i];
              currentVoltage = previousVoltageSetLow[i];
            } else {
              previousVoltageSetLow[i] += voltageStepSizeLow[i];
              previousVoltageSetHigh[i] += voltageStepSizeHigh[i];
              currentVoltage = previousVoltageSetHigh[i];
            }
            DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                         currentVoltage);
          }
        }
        DACController::toggleLdac();
        steps++;
        adcGetsSinceLastDacSet = 0;
        TimingUtil::dacFlag = false;
        TIM8->CNT = 0;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
    }

    PeripheralCommsController::dataLedOff();

    delete[] dacChannels;
    delete[] dacV0_1;
    delete[] dacVf_1;
    delete[] dacV0_2;
    delete[] dacVf_2;
    delete[] adcChannels;
    delete[] voltageStepSizeLow;
    delete[] previousVoltageSetLow;
    delete[] voltageStepSizeHigh;
    delete[] previousVoltageSetHigh;

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }
};
