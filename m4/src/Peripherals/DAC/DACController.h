#pragma once

#include <Arduino.h>
#include <Config.h>
#include <SPI.h>

#include <vector>

#include "DACChannel.h"
#include "FunctionRegistry.h"
#include "FunctionRegistryMacros.h"
#include "Peripherals/OperationResult.h"

class DACController {
 private:
  inline static std::vector<DACChannel*> dac_channels;

  static PeripheralCommsController* commsController;


 public:

  static void initializeRegistry() {
    REGISTER_MEMBER_FUNCTION_0(initialize, "INITIALIZE");
    REGISTER_MEMBER_FUNCTION_0(initialize, "INIT");
    REGISTER_MEMBER_FUNCTION_2(setVoltage, "SET");
    REGISTER_MEMBER_FUNCTION_1(getVoltage, "GET_DAC");
  }

  static void addChannel(int cs_pin, int ldac_pin) {
    DACChannel* newChannel = new DACChannel(commsController, cs_pin, ldac_pin);
    dac_channels.push_back(newChannel);
  }

  static OperationResult initialize() {
    for (auto channel : dac_channels) {
      channel->initialize();
    }
    return OperationResult::Success("INITIALIZATION COMPLETE");
  }

  static void setup() {
    initializeRegistry();
    for (auto channel : dac_channels) {
      channel->setup();
    }
  }

  static DACChannel* getChannel(int channel_index) {
    if (!isChannelIndexValid(channel_index)) {
      return nullptr;
    }
    return dac_channels[channel_index];
  }

  static bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) < dac_channels.size();
  }

  static OperationResult setVoltage(int channel_index, float voltage) {
    DACChannel* dac_channel = dac_channels[channel_index];
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel_index));
    }
    if (voltage < dac_channel->getLowerBound() ||
        voltage > dac_channel->getUpperBound()) {
      return OperationResult::Failure("Voltage out of bounds for DAC " +
                                      String(channel_index));
    }
    float v = dac_channel->setVoltage(voltage);
    return OperationResult::Success("DAC " + String(channel_index) +
                                    " UPDATED TO " + String(v, 6) + " V");
  }

  static float setVoltageBuffer(int channel_index, float voltage) {
    if (!isChannelIndexValid(channel_index)) {
      return -1;
    }
    return dac_channels[channel_index]->setVoltageBuffer(voltage);
  }

  static OperationResult getVoltage(int channel_index) {
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel_index));
    }
    return OperationResult::Success(
        String(dac_channels[channel_index]->getVoltage(), 6));
  }

  static void setCalibration(int channel_index, float offset, float gain) {
    if (!isChannelIndexValid(channel_index)) {
      return;
    }

    dac_channels[channel_index]->setCalibration(offset, gain);
  }

  static float getLowerBound(int channel) {
    if (!isChannelIndexValid(channel)) {
      return -1;
    }
    return dac_channels[channel]->getLowerBound();
  }

  static float getUpperBound(int channel) {
    if (!isChannelIndexValid(channel)) {
      return -1;
    }
    return dac_channels[channel]->getUpperBound();
  }

  static void sendCode(int channel, int code) {
    if (!isChannelIndexValid(channel)) {
      return;
    }
    dac_channels[channel]->sendCode(code);
  }
};

PeripheralCommsController* DACController::commsController = new PeripheralCommsController(DAC_SPI_SETTINGS);