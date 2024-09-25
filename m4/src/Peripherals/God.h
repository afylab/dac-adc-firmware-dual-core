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

    // Check if we have enough arguments for all DAC and ADC channels
    if (args.size() !=
        static_cast<size_t>(5 + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Allocate memory for DAC and ADC channel information
    int* dacChannels = new int[numDacChannels];
    float* dacV0s = new float[numDacChannels];
    float* dacVfs = new float[numDacChannels];
    int* adcChannels = new int[numAdcChannels];

    // Parse DAC channel information
    for (int i = 0; i < numDacChannels; ++i) {
      int baseIndex = 5 + i * 3;
      dacChannels[i] = static_cast<int>(args[baseIndex]);
      dacV0s[i] = static_cast<float>(args[baseIndex + 1]);
      dacVfs[i] = static_cast<float>(args[baseIndex + 2]);
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
    // uint32_t adc_comms_period_us = (1.0/SPI_SPEED)*1e6*8*4; // 8 bits per
    // byte, 4 bytes per ADC conversion if (adc_interval_us <
    // adc_comms_period_us*numAdcChannels) {
    //   return OperationResult::Failure("ADC interval too short");
    // }
    // uint32_t dac_comms_period_us = (1.0/SPI_SPEED)*1e6*8*3; // 8 bits per
    // byte, 3 bytes per DAC update if (dac_interval_us <
    // dac_comms_period_us*numDacChannels) {
    //   return OperationResult::Failure("DAC interval too short");
    // }

    int steps = 0;
    int x = 0;

    const int saved_data_size = numSteps * dac_interval_us / adc_interval_us;

    float** voltSetpoints = new float*[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltSetpoints[i] = new float[numSteps];
      for (int j = 0; j < numSteps; j++) {
        voltSetpoints[i][j] =
            dacV0s[i] + (dacVfs[i] - dacV0s[i]) * j / (numSteps - 1);
      }
    }

    TimingUtil::setupTimersTimeSeries(dac_interval_us, adc_interval_us);

    setStopFlag(false);

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
    }

    while (x < saved_data_size && !getStopFlag()) {
      if (TimingUtil::adcFlag) {
        ADCBoard::commsController.beginTransaction();
        if (steps <= 1) {
          for (int i = 0; i < numAdcChannels; i++) {
            ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
        } else {
          VoltagePacket* packets = new VoltagePacket[numAdcChannels];
          for (int i = 0; i < numAdcChannels; i++) {
            float v =
                ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            packets[i] = {static_cast<uint8_t>(adcChannels[i]),
                          static_cast<uint32_t>(x), v};
          }
          m4SendVoltage(packets, numAdcChannels);
          delete[] packets;
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

    for (int i = 0; i < numDacChannels; i++) {
      delete[] voltSetpoints[i];
    }
    delete[] voltSetpoints;

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

    // Check if we have enough arguments for all DAC and ADC channels
    if (args.size() !=
        static_cast<size_t>(6 + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Allocate memory for DAC and ADC channel information
    int* dacChannels = new int[numDacChannels];
    float* dacV0s = new float[numDacChannels];
    float* dacVfs = new float[numDacChannels];
    int* adcChannels = new int[numAdcChannels];

    // Parse DAC channel information
    for (int i = 0; i < numDacChannels; ++i) {
      int baseIndex = 6 + i * 3;
      dacChannels[i] = static_cast<int>(args[baseIndex]);
      dacV0s[i] = static_cast<float>(args[baseIndex + 1]);
      dacVfs[i] = static_cast<float>(args[baseIndex + 2]);
    }

    // Parse ADC channel information
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
    // uint32_t adc_comms_period_us = (1.0/SPI_SPEED)*1e6*8*4; // 8 bits per
    // byte, 4 bytes per ADC conversion if
    // (numAdcChannels*numAdcAverages*adc_comms_period_us > dac_interval_us -
    // dac_settling_time_us) {
    //   return OperationResult::Failure("Buffer Ramp limited by ADC SPI
    //   comms");
    // }

    int steps = 0;
    int x = 0;

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

    setStopFlag(false);

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
    }
    while (x < numSteps && !getStopFlag()) {
      if (TimingUtil::adcFlag) {
        ADCBoard::commsController.beginTransaction();
        if (steps <= 1) {
          for (int i = 0; i < numAdcChannels; i++) {
            for (int j = 0; j < numAdcAverages; j++) {
              ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            }
          }
        } else {
          VoltagePacket* packets = new VoltagePacket[numAdcChannels];
          for (int i = 0; i < numAdcChannels; i++) {
            float total = 0.0;
            for (int j = 0; j < numAdcAverages; j++) {
              total +=
                  ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            }
            float v = total / numAdcAverages;
            packets[i] = {static_cast<uint8_t>(adcChannels[i]),
                          static_cast<uint32_t>(x), v};
          }
          m4SendVoltage(packets, numAdcChannels);
          delete[] packets;
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

    for (int i = 0; i < numDacChannels; i++) {
      delete[] voltSetpoints[i];
    }
    delete[] voltSetpoints;

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
};