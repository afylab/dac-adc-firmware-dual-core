#pragma once

#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "FunctionRegistry.h"
#include "FunctionRegistryMacros.h"
#include "Peripherals/ADC/ADCBoard.h"
#include "Peripherals/OperationResult.h"
class ADCController {
 private:
  inline static std::vector<ADCBoard> adc_boards;

 public:
  static OperationResult initialize() {
    // for (auto channel : adc_channels) {
    //   channel->initialize();
    // }
    return OperationResult::Success();
  }

  static void setup() {
    initializeRegistry();
    for (auto board : adc_boards) {
      board.setup();
    }
  }

  static void initializeRegistry() {
    REGISTER_MEMBER_FUNCTION_1(readChannelVoltage, "GET_ADC");
    REGISTER_MEMBER_FUNCTION_2(setConversionTime, "CONVERT_TIME");
    REGISTER_MEMBER_FUNCTION_3(continuousConvertRead, "CONTINUOUS_CONVERT_READ");
    REGISTER_MEMBER_FUNCTION_1(idleMode, "IDLE_MODE");
    REGISTER_MEMBER_FUNCTION_0(getChannelsActive, "GET_CHANNELS_ACTIVE");
    REGISTER_MEMBER_FUNCTION_0(resetAllADCBoards, "RESET");
    REGISTER_MEMBER_FUNCTION_1(talkADC, "TALK");
    REGISTER_MEMBER_FUNCTION_0(adcZeroScaleCal, "ADC_ZERO_SC_CAL");
    REGISTER_MEMBER_FUNCTION_0(adcChannelSystemZeroScaleCal,
                               "ADC_CH_ZERO_SC_CAL");
    REGISTER_MEMBER_FUNCTION_0(adcChannelSystemFullScaleCal,
                               "ADC_CH_FULL_SC_CAL");
  }

  static void addBoard(int data_sync_pin, int data_ready, int reset_pin) {
    ADCBoard newBoard = ADCBoard(data_sync_pin, data_ready, reset_pin);
    adc_boards.push_back(newBoard);
  }

  static bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) <
               adc_boards.size() * NUM_CHANNELS_PER_ADC_BOARD;
  }

  #define getBoardIndexFromGlobalIndex(channel_index) channel_index / NUM_CHANNELS_PER_ADC_BOARD

  #define getChannelIndexFromGlobalIndex(channel_index) channel_index % NUM_CHANNELS_PER_ADC_BOARD

  static OperationResult readChannelVoltage(int channel_index) {
    if (isChannelIndexValid(channel_index)) {
      return OperationResult::Success(String(getVoltage(channel_index), 9));
    } else {
      return OperationResult::Failure("Invalid channel index");
    }
  }

  static float getVoltage(int channel_index) {
    return adc_boards[getBoardIndexFromGlobalIndex(channel_index)].readVoltage(
        getChannelIndexFromGlobalIndex(channel_index));
  }

  static uint32_t getConversionData(int adc_channel) {
    return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        .getConversionData(getChannelIndexFromGlobalIndex(adc_channel));
  }

  static float getVoltageData(int adc_channel) {
    return ADC2DOUBLE(getConversionData(adc_channel));
  }

   static float getVoltageDataNoTransaction(int adc_channel) {
    return ADC2DOUBLE(adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        .getConversionDataNoTransaction(getChannelIndexFromGlobalIndex(adc_channel)));
  }

  static void startContinuousConversion(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        .startContinuousConversion(getChannelIndexFromGlobalIndex(adc_channel));
  }

  static OperationResult continuousConvertRead(int channel_index,
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

  static OperationResult idleMode(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].idleMode(
        getChannelIndexFromGlobalIndex(adc_channel));
    return OperationResult::Success("Returned ADC " + String(adc_channel) +
                                    " to idle mode");
  }

  static OperationResult getChannelsActive() {
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

  static String parseVector(std::vector<double> data) {
    String result = "";
    for (auto d : data) {
      result += String(d, 9) + ",";
    }
    return result.substring(0, result.length() - 1);
  }
  static String parseVector(std::vector<bool> data) {
    String result = "";
    for (auto d : data) {
      result += String(d) + ",";
    }
    return result.substring(0, result.length() - 1);
  }
  static String parseVector(std::vector<int> data) {
    String result = "";
    for (auto d : data) {
      result += String(d) + ",";
    }
    return result.substring(0, result.length() - 1);
  }

  static OperationResult resetAllADCBoards() {
    for (auto board : adc_boards) {
      board.reset();
    }
    return OperationResult::Success();
  }

  static OperationResult talkADC(byte command) {
    String results = "";
    for (auto board : adc_boards) {
      results += String(board.talkADC(command), 9) + "\n";
    }
    return OperationResult::Success(results);
  }

  static OperationResult adcZeroScaleCal() {
    for (auto board : adc_boards) {
      board.zeroScaleSelfCalibration();
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  static OperationResult adcChannelSystemZeroScaleCal() {
    for (auto board : adc_boards) {
      for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
        board.zeroScaleChannelSystemSelfCalibration(i);
      }
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  static OperationResult adcChannelSystemFullScaleCal() {
    for (auto board : adc_boards) {
      for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
        board.fullScaleChannelSystemSelfCalibration(i);
      }
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  static OperationResult setConversionTime(int adc_channel, int time_us) {
    float setpoint =
        adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].setConversionTime(
            getChannelIndexFromGlobalIndex(adc_channel), time_us);
    if (setpoint == -1.0) {
      return OperationResult::Failure(
          "The filter word you selected is not valid.");
    }
    return OperationResult::Success(String(setpoint, 9));
  }
};