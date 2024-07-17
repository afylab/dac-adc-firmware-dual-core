#pragma once

#include <Arduino.h>
#include <Config.h>
#include <SPI.h>

#include <vector>

#include "DACChannel.h"
#include "FunctionRegistry.h"
#include "FunctionRegistryMacros.h"
#include "Peripherals/OperationResult.h"
#include "Peripherals/Peripheral.h"

class DACController : public Peripheral {
 private:
  std::vector<DACChannel*> dac_channels;

 public:
  float DAC_FULL_SCALE = DEFAULT_DAC_FULL_SCALE;

  DACController(FunctionRegistry& r, PeripheralCommsController& c)
      : Peripheral(r, c) {}

  ~DACController() {
    for (auto dac : dac_channels) {
      delete dac;
    }
  }

  void initializeRegistry() override {
    REGISTER_MEMBER_FUNCTION_0(registry, initialize, "INITIALIZE");
    REGISTER_MEMBER_FUNCTION_0(registry, initialize, "INIT");
    REGISTER_MEMBER_FUNCTION_2(registry, setVoltage, "SET");
    REGISTER_MEMBER_FUNCTION_1(registry, getVoltage, "GET_DAC");
  }

  void addChannel(int cs_pin, int ldac_pin) {
    DACChannel* newChannel = new DACChannel(commsController, cs_pin, ldac_pin);
    dac_channels.push_back(newChannel);
  }

  OperationResult initialize() override {
    for (auto channel : dac_channels) {
      channel->initialize();
    }
    return OperationResult::Success("INITIALIZATION COMPLETE");
  }

  void setup() override {
    initializeRegistry();
    for (auto channel : dac_channels) {
      channel->setup();
    }
  }

  bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) < dac_channels.size();
  }

  OperationResult setVoltage(int channel_index, float voltage) {
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

  float setVoltageBuffer(int channel_index, float voltage) {
    if (!isChannelIndexValid(channel_index)) {
      return -1;
    }
    return dac_channels[channel_index]->setVoltageBuffer(voltage);
  }

  OperationResult getVoltage(int channel_index) {
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel_index));
    }
    return OperationResult::Success(
        String(dac_channels[channel_index]->getVoltage(), 6));
  }

  void setCalibration(int channel_index, float offset, float gain) {
    if (!isChannelIndexValid(channel_index)) {
      return;
    }

    dac_channels[channel_index]->setCalibration(offset, gain);
  }

  float getLowerBound(int channel) {
    if (!isChannelIndexValid(channel)) {
      return -1;
    }
    return dac_channels[channel]->getLowerBound();
  }

  float getUpperBound(int channel) {
    if (!isChannelIndexValid(channel)) {
      return -1;
    }
    return dac_channels[channel]->getUpperBound();
  }

  void sendCode(int channel, int code) {
    if (!isChannelIndexValid(channel)) {
      return;
    }
    dac_channels[channel]->sendCode(code);
  }
};