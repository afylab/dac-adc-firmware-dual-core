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
    REGISTER_MEMBER_FUNCTION_VECTOR(timeSeriesBufferRampWrapper,
                                    "TIME_SERIES_BUFFER_RAMP");
    REGISTER_MEMBER_FUNCTION_VECTOR(dacLedBufferRampWrapper,
                                    "DAC_LED_BUFFER_RAMP");
    REGISTER_MEMBER_FUNCTION_0(dacChannelCalibration, "DAC_CH_CAL");
    REGISTER_MEMBER_FUNCTION_VECTOR(twoDimensionalFlexibleRampWrapper,
                                    "2D_TIME_SERIES_RAMP");
  }

  struct AxisChannel {
    int channel;
    float v0;
    float vf;
  };

  struct Axis {
    std::vector<AxisChannel> channels;
  };

  // args:
  // numDacChannels, numAdcChannels, retrace, numSteps_slow, numSteps_fast,
  // dac_interval_us, adc_interval_us,
  // slow_axis_num_channels, slow_ch1, slow_v01, slow_vf1, slow_ch2, slow_v02,
  // slow_vf2, ..., fast_axis_num_channels, fast_ch1, fast_v01, fast_vf1,
  // fast_ch2, fast_v02, fast_vf2, ..., adc0, adc1, adc2, ...
  static OperationResult twoDimensionalFlexibleRampWrapper(
      const std::vector<float>& args) {
    if (args.size() < 9) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    int numDacChannels = static_cast<int>(args[0]);
    int numAdcChannels = static_cast<int>(args[1]);
    bool retrace = static_cast<bool>(args[2]);
    int numSteps_slow = static_cast<int>(args[3]);
    int numSteps_fast = static_cast<int>(args[4]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[5]);
    uint32_t adc_interval_us = static_cast<uint32_t>(args[6]);

    int slow_axis_num_channels = static_cast<int>(args[7]);

    // Parse slow axis information
    Axis slow_axis;
    for (int i = 0; i < slow_axis_num_channels; ++i) {
      int base_index = 8 + i * 3;
      AxisChannel channel = {static_cast<int>(args[base_index]),
                             args[base_index + 1], args[base_index + 2]};
      slow_axis.channels.push_back(channel);
    }

    // Parse fast axis information
    int fast_axis_start = 8 + slow_axis_num_channels * 3;
    int fast_axis_num_channels = static_cast<int>(args[fast_axis_start]);
    Axis fast_axis;
    for (int i = 0; i < fast_axis_num_channels; ++i) {
      int base_index = fast_axis_start + 1 + i * 3;
      AxisChannel channel = {static_cast<int>(args[base_index]),
                             args[base_index + 1], args[base_index + 2]};
      fast_axis.channels.push_back(channel);
    }

    // Parse ADC channel information
    std::vector<int> adcChannels(numAdcChannels);
    int adc_start = fast_axis_start + 1 + fast_axis_num_channels * 3;
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[adc_start + i]);
    }

    // print all info for debugging
    char buffer[1000];
    sprintf(
        buffer,
        "numDacChannels: %d\nnumAdcChannels: %d\nretrace: %d\nnumSteps_slow: "
        "%d\nnumSteps_fast: %d\ndac_interval_us: %d\nadc_interval_us: "
        "%d\nslow_axis_num_channels: %d\n",
        numDacChannels, numAdcChannels, retrace, numSteps_slow, numSteps_fast,
        dac_interval_us, adc_interval_us, slow_axis_num_channels);
    m4SendChar(buffer, strlen(buffer));
    for (int i = 0; i < slow_axis_num_channels; ++i) {
      int base_index = 8 + i * 3;
      sprintf(buffer, "slow_axis channel: %d, v0: %f, vf: %f\n",
              static_cast<int>(args[base_index]), args[base_index + 1],
              args[base_index + 2]);
      m4SendChar(buffer, strlen(buffer));
    }

    return twoDimensionalFlexibleRampBase(
        numDacChannels, numAdcChannels, retrace, numSteps_slow, numSteps_fast,
        dac_interval_us, adc_interval_us, slow_axis, fast_axis, adcChannels);
  }

  static OperationResult twoDimensionalFlexibleRampBase(
      int numDacChannels, int numAdcChannels, bool retrace, int numSteps_slow,
      int numSteps_fast, uint32_t dac_interval_us, uint32_t adc_interval_us,
      const Axis& slow_axis, const Axis& fast_axis,
      std::vector<int>& adcChannels) {
    if (adc_interval_us < 1 || dac_interval_us < 1) {
      return OperationResult::Failure("Invalid interval");
    }

    std::unordered_set<int> uniqueChannels;
    std::vector<int> dacChannels;
    std::vector<float> dacV0s(numDacChannels, 0.0f);
    std::vector<float> dacVfs(numDacChannels, 0.0f);

    // Collect unique DAC channels
    for (const auto& ch : slow_axis.channels) {
      if (uniqueChannels.insert(ch.channel).second) {
        dacChannels.push_back(ch.channel);
      }
    }
    for (const auto& ch : fast_axis.channels) {
      if (uniqueChannels.insert(ch.channel).second) {
        dacChannels.push_back(ch.channel);
      }
    }

    bool forward = true;
    for (int slow_step = 0; slow_step < numSteps_slow; ++slow_step) {
      float slow_progress = static_cast<float>(slow_step) / (numSteps_slow - 1);

      // Set voltages for slow axis channels
      for (const auto& ch : slow_axis.channels) {
        float voltage = ch.v0 + (ch.vf - ch.v0) * slow_progress;
        dacV0s[ch.channel] = voltage;
        dacVfs[ch.channel] = voltage;
      }

      // Set initial and final voltages for fast axis channels
      for (const auto& ch : fast_axis.channels) {
        if (forward) {
          dacV0s[ch.channel] = ch.v0;
          dacVfs[ch.channel] = ch.vf;
        } else {
          dacV0s[ch.channel] = ch.vf;
          dacVfs[ch.channel] = ch.v0;
        }
      }

      // print the slow axis
      char buffer[100];
      sprintf(buffer, "Slow axis step %d/%d, Direction: %s", slow_step,
              numSteps_slow, forward ? "Forward" : "Reverse");
      m4SendChar(buffer, strlen(buffer));
      // print all data you set for debugging
      for (int i = 0; i < numDacChannels; ++i) {
        sprintf(buffer, "DAC channel %d: v0: %f, vf: %f\n", dacChannels[i],
                dacV0s[i], dacVfs[i]);
        m4SendChar(buffer, strlen(buffer));
      }

      // Call the 1D ramp for the fast axis
      OperationResult result = timeSeriesBufferRampBase(
          dacChannels.size(), numAdcChannels, numSteps_fast, dac_interval_us,
          adc_interval_us, dacChannels.data(), dacV0s.data(), dacVfs.data(),
          adcChannels.data());

      if (retrace) {
        forward = !forward;
      }

      delayMicroseconds(1000);
    }

    return OperationResult::Success(
        "2D Flexible Axis Ramp completed successfully");
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
          VoltagePacket* packets = new VoltagePacket[numAdcChannels];
          for (int i = 0; i < numAdcChannels; i++) {
            float v =
                ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            packets[i] = {static_cast<uint8_t>(adcChannels[i]),
                          static_cast<uint32_t>(x), v};
          }
          m4SendVoltage(packets, numAdcChannels);
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

    return OperationResult::Success();
  }

  // args:
  // numDacChannels, numAdcChannels, numSteps, dacInterval_us,
  // dacSettlingTime_us, dacchannel0, dacv00, dacvf0, dacchannel1, dacv01,
  // dacvf1, ..., adc0, adc1, adc2, ...
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

    return dacLedBufferRampBase(numDacChannels, numAdcChannels, numSteps,
                                dac_interval_us, dac_settling_time_us,
                                dacChannels, dacV0s, dacVfs, adcChannels);
  }

  static OperationResult dacLedBufferRampBase(int numDacChannels,
                                              int numAdcChannels, int numSteps,
                                              uint32_t dac_interval_us,
                                              uint32_t dac_settling_time_us,
                                              int* dacChannels, float* dacV0s,
                                              float* dacVfs, int* adcChannels) {
    if (dac_settling_time_us < 1 || dac_interval_us < 1 ||
        dac_settling_time_us >= dac_interval_us) {
      return OperationResult::Failure("Invalid interval or settling time");
    }

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
          VoltagePacket* packets = new VoltagePacket[numAdcChannels];
          for (int i = 0; i < numAdcChannels; i++) {
            float v =
                ADCController::getVoltageDataNoTransaction(adcChannels[i]);
            packets[i] = {static_cast<uint8_t>(adcChannels[i]),
                          static_cast<uint32_t>(x), v};
          }
          m4SendVoltage(packets, numAdcChannels);
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