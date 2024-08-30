#pragma once

#include <Arduino.h>

#include <vector>

#include "FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

// #include "RPC.h"

struct UserIOHandler {
  static void setup() {
    // RPC.begin();
    REGISTER_MEMBER_FUNCTION_0(nop, "NOP");
    REGISTER_MEMBER_FUNCTION_0(id, "*IDN?");
    REGISTER_MEMBER_FUNCTION_0(rdy, "*RDY?");
    REGISTER_MEMBER_FUNCTION_0(serialNumber, "SERIAL_NUMBER");
  }

  static OperationResult nop() { return OperationResult::Success("NOP"); }
  static OperationResult id() {
    return OperationResult::Success("DAC-ADC_AD7734-AD5791");
  }
  static OperationResult rdy() { return OperationResult::Success("READY"); }
  static OperationResult serialNumber() {
    return OperationResult::Success("DA20_16_08");
  }

  static std::vector<String> query_memory() {
    std::vector<String> comm;
    if (!m4CheckForNewData()) {
      return comm;
    }
    char buffer[MESSAGE_SIZE];
    m4GetData(buffer);
    String input = String(buffer);
    input.trim();
    int startPos = 0;
    int commaPos = input.indexOf(',');
    while (commaPos != -1) {
      comm.push_back(input.substring(startPos, commaPos));
      startPos = commaPos + 1;
      commaPos = input.indexOf(',', startPos);
    }
    comm.push_back(input.substring(startPos));

    return comm;
  }

  static bool isValidFloat(const String& str) {
    char* endPtr;
    strtod(str.c_str(), &endPtr);
    return endPtr != str.c_str() && *endPtr == '\0' && str.length() > 0;
  }

  static void handleUserIO() {
    std::vector<String> comm;
    if (m4CheckForNewData()) {
      comm = query_memory();

      if (comm.size() > 0) {
        String command = comm[0];
        std::vector<float> args;

        for (size_t i = 1; i < comm.size(); ++i) {
          if (!isValidFloat(comm[i])) {
            m4SendData("Invalid arguments!");
            return;
          }
          args.push_back(comm[i].toFloat());
        }
        OperationResult result =
            OperationResult::Failure("Something went wrong!");
        FunctionRegistry::ExecuteResult executeResult =
            FunctionRegistry::execute(command, args, result);

        switch (executeResult) {
          case FunctionRegistry::ExecuteResult::Success:
            if (result.hasMessage()) {
              char* message = new char[result.getMessage().length() + 1];
              result.getMessage().toCharArray(message,
                                              result.getMessage().length() + 1);
              m4SendData(message);
              delete[] message;
            }
            break;
          case FunctionRegistry::ExecuteResult::ArgumentError:
            m4SendData("Error: Argument error");
            break;
          case FunctionRegistry::ExecuteResult::FunctionNotFound:
            m4SendData("Error: Function not found");
            break;
        }
      }
      // m4SendData(getDebugBuffer());
    }
  }
};