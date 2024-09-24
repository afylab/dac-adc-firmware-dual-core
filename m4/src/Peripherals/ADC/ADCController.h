#pragma once

#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "FunctionRegistry/FunctionRegistry.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCBoard.h"
#include "Peripherals/OperationResult.h"
class ADCController {
 private:
  inline static std::vector<ADCBoard> adc_boards;

 public:
  inline static OperationResult initialize() {
    // for (auto channel : adc_channels) {
    //   channel->initialize();
    // }
    return OperationResult::Success();
  }

  inline static void setup() {
    initializeRegistry();
    for (auto board : adc_boards) {
      board.setup();
    }
  }

  static void initializeRegistry() {
    registerMemberFunction(readChannelVoltage, "GET_ADC");
    registerMemberFunction(setConversionTime, "CONVERT_TIME");
    registerMemberFunction(getConversionTime, "GET_CONVERT_TIME");
    registerMemberFunction(continuousConvertRead,
                               "CONTINUOUS_CONVERT_READ");
    registerMemberFunction(idleMode, "IDLE_MODE");
    registerMemberFunction(getChannelsActive, "GET_CHANNELS_ACTIVE");
    registerMemberFunction(resetAllADCBoards, "RESET");
    registerMemberFunction(talkADC, "TALK");
    registerMemberFunction(adcZeroScaleCal, "ADC_ZERO_SC_CAL");
    registerMemberFunction(adcChannelSystemZeroScaleCal,
                               "ADC_CH_ZERO_SC_CAL");
    registerMemberFunction(adcChannelSystemFullScaleCal,
                               "ADC_CH_FULL_SC_CAL");
  }

  inline static void addBoard(int data_sync_pin, int data_ready,
                              int reset_pin) {
    ADCBoard newBoard = ADCBoard(data_sync_pin, data_ready, reset_pin);
    adc_boards.push_back(newBoard);
  }

  inline static bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) <
               adc_boards.size() * NUM_CHANNELS_PER_ADC_BOARD;
  }

#define getBoardIndexFromGlobalIndex(channel_index) \
  channel_index / NUM_CHANNELS_PER_ADC_BOARD

#define getChannelIndexFromGlobalIndex(channel_index) \
  channel_index % NUM_CHANNELS_PER_ADC_BOARD

  inline static OperationResult readChannelVoltage(int channel_index) {
    if (isChannelIndexValid(channel_index)) {
      return OperationResult::Success(String(getVoltage(channel_index), 9));
    } else {
      return OperationResult::Failure("Invalid channel index");
    }
  }

  inline static float getVoltage(int channel_index) {
    return adc_boards[getBoardIndexFromGlobalIndex(channel_index)].readVoltage(
        getChannelIndexFromGlobalIndex(channel_index));
  }

  inline static uint32_t getConversionData(int adc_channel) {
    return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        .getConversionData(getChannelIndexFromGlobalIndex(adc_channel));
  }

  inline static float getVoltageData(int adc_channel) {
    return ADC2DOUBLE(getConversionData(adc_channel));
  }

  inline static float getVoltageDataNoTransaction(int adc_channel) {
    return ADC2DOUBLE(adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
                          .getConversionDataNoTransaction(
                              getChannelIndexFromGlobalIndex(adc_channel)));
  }

  inline static void startContinuousConversion(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        .startContinuousConversion(getChannelIndexFromGlobalIndex(adc_channel));
  }

  inline static OperationResult continuousConvertRead(int channel_index,
                                                      uint32_t frequency_us,
                                                      uint32_t duration_us) {
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index");
    }
    if (frequency_us < 1) {
      return OperationResult::Failure("Invalid frequency");
    }
    if (duration_us < 1) {
      return OperationResult::Failure("Invalid duration");
    }
    if (frequency_us > duration_us) {
      return OperationResult::Failure("Frequency must be less than duration");
    }

    std::vector<double> data =
        adc_boards[getBoardIndexFromGlobalIndex(channel_index)]
            .continuousConvert(getChannelIndexFromGlobalIndex(channel_index),
                               frequency_us, duration_us);
    String result = "";
    for (auto d : data) {
      result += String(d, 9) + ",";
    }
    result = result.substring(0, result.length() - 1);

    return OperationResult::Success(result);
  }

  inline static OperationResult idleMode(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].idleMode(
        getChannelIndexFromGlobalIndex(adc_channel));
    return OperationResult::Success("Returned ADC " + String(adc_channel) +
                                    " to idle mode");
  }

  inline static OperationResult getChannelsActive() {
    std::vector<int> statuses;
    for (auto board : adc_boards) {
      for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
        if (board.isChannelActive(i)) {
          statuses.push_back(i);
        }
      }
    }
    String output = parseVector(statuses);
    return OperationResult::Success(output == "" ? "NONE" : output);
  }

  template <typename T>
  inline static String parseVector(const std::vector<T>& data) {
    String result = "";
    for (const auto& d : data) {
      result += String(d) + ",";
    }
    return result.substring(0, result.length() - 1);
  }

  inline static OperationResult resetAllADCBoards() {
    for (auto board : adc_boards) {
      board.reset();
    }
    return OperationResult::Success();
  }

  inline static OperationResult talkADC(byte command) {
    String results = "";
    for (auto board : adc_boards) {
      results += String(board.talkADC(command), 9) + "\n";
    }
    return OperationResult::Success(results);
  }

  inline static OperationResult adcZeroScaleCal() {
    for (auto board : adc_boards) {
      board.zeroScaleSelfCalibration();
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  inline static OperationResult adcChannelSystemZeroScaleCal() {
    for (auto board : adc_boards) {
      for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
        board.zeroScaleChannelSystemSelfCalibration(i);
      }
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  inline static OperationResult adcChannelSystemFullScaleCal() {
    for (auto board : adc_boards) {
      for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
        board.fullScaleChannelSystemSelfCalibration(i);
      }
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  static OperationResult setConversionTime(int adc_channel, int time_us) {
    if (!isChannelIndexValid(adc_channel)) {
      return OperationResult::Failure("Invalid channel index");
    }
    float setpoint =
        adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].setConversionTime(
            getChannelIndexFromGlobalIndex(adc_channel), time_us);
    if (setpoint == -1.0) {
      return OperationResult::Failure(
          "The filter word you selected is not valid.");
    }
    return OperationResult::Success(String(setpoint, 9));
  }

  static float getConversionTimeFloat(int adc_channel) {
    if (!isChannelIndexValid(adc_channel)) {
      return -1.0;
    }
    return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].getConversionTime(
        getChannelIndexFromGlobalIndex(adc_channel));
  }

  static OperationResult getConversionTime(int adc_channel) {
    if (!isChannelIndexValid(adc_channel)) {
      return OperationResult::Failure("Invalid channel index");
    }
    float convert_time =
        adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].getConversionTime(
            getChannelIndexFromGlobalIndex(adc_channel));
    return OperationResult::Success(String(convert_time, 9));
  }
};