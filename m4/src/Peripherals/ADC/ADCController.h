#pragma once

#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "Peripherals/ADC/ADCBoard.h"

class ADCController {
 private:
  inline static std::vector<ADCBoard*> adc_boards;

  static ADCController* instance;
  static PeripheralCommsController* commsController;

 public:
  ADCController(FunctionRegistry& r, PeripheralCommsController& c) {
        instance = this;
      }

  static OperationResult initialize() {
    // for (auto channel : adc_channels) {
    //   channel->initialize();
    // }
    return OperationResult::Success();
  }

  static void setup() {
    initializeRegistry();
    for (auto board : adc_boards) {
      board->setup();
    }
  }

  static void initializeRegistry() {
    REGISTER_MEMBER_FUNCTION_1(readChannelVoltage, "GET_ADC");
    REGISTER_MEMBER_FUNCTION_3(setConversionTime, "SET_CONVERSION_TIME");
    REGISTER_MEMBER_FUNCTION_3(continuousConvert, "CONTINUOUS_CONVERT");
    REGISTER_MEMBER_FUNCTION_1(idleMode, "IDLE_MODE");
    REGISTER_MEMBER_FUNCTION_0(getChannelsActive, "GET_CHANNELS_ACTIVE");
  }

  static void addBoard(int data_sync_pin, int data_ready, int reset_pin) {
    ADCBoard* newBoard =
        new ADCBoard(commsController, data_sync_pin, data_ready, reset_pin);
    adc_boards.push_back(newBoard);
  }

  static bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) <
               adc_boards.size() * NUM_CHANNELS_PER_ADC_BOARD;
  }

  static int getBoardIndexFromGlobalIndex(int channel_index) {
    return channel_index / NUM_CHANNELS_PER_ADC_BOARD;
  }

  static int getChannelIndexFromGlobalIndex(int channel_index) {
    return channel_index % NUM_CHANNELS_PER_ADC_BOARD;
  }

  static OperationResult readChannelVoltage(int channel_index) {
    if (isChannelIndexValid(channel_index)) {
      return OperationResult::Success(String(
          adc_boards[getBoardIndexFromGlobalIndex(channel_index)]->readVoltage(
              getChannelIndexFromGlobalIndex(channel_index)), 6));
    } else {
      return OperationResult::Failure("Invalid channel index");
    }
  }

  static uint16_t getConversionData(int adc_channel) {
    return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        ->getConversionData(getChannelIndexFromGlobalIndex(adc_channel));
  }

  static OperationResult setConversionTime(int adc_channel, bool chop, int fw) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        ->setConversionTime(getChannelIndexFromGlobalIndex(adc_channel), chop,
                            fw);
    return OperationResult::Success("Done");
  }

  static float getVoltageData(int adc_channel) {
    return ADC2DOUBLE(getConversionData(adc_channel));
  }

  static void startContinuousConversion(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        ->startContinuousConversion(getChannelIndexFromGlobalIndex(adc_channel));
  }

  static OperationResult continuousConvert(int channel_index, uint32_t frequency_us,
                                     uint32_t duration) {
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index");
    }
    if (frequency_us < 1) {
      return OperationResult::Failure("Invalid frequency");
    }

    std::vector<double> data = adc_boards[getBoardIndexFromGlobalIndex(channel_index)]
        ->continuousConvert(getChannelIndexFromGlobalIndex(channel_index),
                             frequency_us, duration);
    String result = "";
    for (auto d : data) {
      result += String(d, 6) + ",";
    }
    result = result.substring(0, result.length() - 1);

    return OperationResult::Success(result);
  }

  static OperationResult idleMode(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        ->idleMode(getChannelIndexFromGlobalIndex(adc_channel));
    return OperationResult::Success("Returned ADC " + String(adc_channel) + " to idle mode");
  }

  static OperationResult getChannelsActive() {
    std::vector<bool> statuses;
    for (auto board : adc_boards) {
      for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
        statuses.push_back(board->isChannelActive(i));
      }
    }
    return OperationResult::Success(parseVector(statuses));
  }

  static String parseVector(std::vector<double> data) {
    String result = "";
    for (auto d : data) {
      result += String(d, 6) + ",";
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
};

ADCController* ADCController::instance = nullptr;
PeripheralCommsController* ADCController::commsController = nullptr;