#pragma once

#include <Arduino.h>
#include <FunctionRegistry.h>
#include <FunctionRegistryMacros.h>

#include "PeripheralCommsController.h"

class Peripheral {
 public:
  FunctionRegistry& registry;
  PeripheralCommsController& commsController;

  Peripheral(FunctionRegistry& r, PeripheralCommsController& c)
      : registry(r), commsController(c) {}

  virtual ~Peripheral() = default;

  // initialize is the command INITIALIZE, setup is called in main::setup
  virtual void setup() = 0;
  virtual OperationResult initialize() = 0;

  virtual void initializeRegistry() = 0;
};