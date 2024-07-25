#pragma once

#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "Peripherals/ADC/ADCBoard.h"
#include "Peripherals/Peripheral.h"

class ADCController : public Peripheral {
 private:
  std::vector<ADCBoard*> adc_boards;

 public:
  ADCController(FunctionRegistry& r, PeripheralCommsController& c)
      : Peripheral(r, c) {}

  OperationResult initialize() override {
    // for (auto channel : adc_channels) {
    //   channel->initialize();
    // }
    // Serial.println("ADC INITIALIZATION COMPLETE");
    return OperationResult::Success();
  }

  void setup() override {
    initializeRegistry();
    for (auto board : adc_boards) {
      board->setup();
    }
  }

  void initializeRegistry() override {
    REGISTER_MEMBER_FUNCTION_1(registry, readChannelVoltage, "GET_ADC");
    REGISTER_MEMBER_FUNCTION_3(registry, setConversionTime, "SET_CONVERSION_TIME");
    REGISTER_MEMBER_FUNCTION_3(registry, continuousConvert, "CONTINUOUS_CONVERT");
    REGISTER_MEMBER_FUNCTION_1(registry, idleMode, "IDLE_MODE");
  }

  void addBoard(int data_sync_pin, int data_ready, int reset_pin) {
    ADCBoard* newBoard =
        new ADCBoard(commsController, data_sync_pin, data_ready, reset_pin);
    adc_boards.push_back(newBoard);
  }

  bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) <
               adc_boards.size() * NUM_CHANNELS_PER_ADC_BOARD;
  }

  int getBoardIndexFromGlobalIndex(int channel_index) {
    return channel_index / NUM_CHANNELS_PER_ADC_BOARD;
  }

  int getChannelIndexFromGlobalIndex(int channel_index) {
    return channel_index % NUM_CHANNELS_PER_ADC_BOARD;
  }

  OperationResult readChannelVoltage(int channel_index) {
    if (isChannelIndexValid(channel_index)) {
      return OperationResult::Success(String(
          adc_boards[getBoardIndexFromGlobalIndex(channel_index)]->readVoltage(
              getChannelIndexFromGlobalIndex(channel_index)), 6));
    } else {
      return OperationResult::Failure("Invalid channel index");
    }
  }

  uint16_t getConversionData(int adc_channel) {
    return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        ->getConversionData(getChannelIndexFromGlobalIndex(adc_channel));
  }

  OperationResult setConversionTime(int adc_channel, bool chop, int fw) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        ->setConversionTime(getChannelIndexFromGlobalIndex(adc_channel), chop,
                            fw);
    return OperationResult::Success("Done");
  }

  float getVoltageData(int adc_channel) {
    return ADC2DOUBLE(getConversionData(adc_channel));
  }

  void startContinuousConversion(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        ->startContinuousConversion(getChannelIndexFromGlobalIndex(adc_channel));
  }

  OperationResult continuousConvert(int channel_index, uint32_t frequency_us,
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

  OperationResult idleMode(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        ->idleMode(getChannelIndexFromGlobalIndex(adc_channel));
    return OperationResult::Success("Returned ADC " + String(adc_channel) + " to idle mode");
  }
};