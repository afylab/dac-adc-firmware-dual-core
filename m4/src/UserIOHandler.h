#pragma once

#include <Arduino.h>

#include <vector>

#include "FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

struct UserIOHandler {
  static void setup() {
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
    if (!m4HasCharMessage()) {
      return comm;
    }

    char buffer[CHAR_BUFFER_SIZE];
    size_t size;
    if (!m4ReceiveChar(buffer, size)) {
      return comm;  // Return empty vector if we couldn't get the data
    }

    String input = String(buffer, size);  // Create String with exact size
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
    if (m4HasCharMessage()) {
      comm = query_memory();

      if (comm.size() > 0) {
        String command = comm[0];
        std::vector<float> args;

        for (size_t i = 1; i < comm.size(); ++i) {
          if (!isValidFloat(comm[i])) {
            m4SendChar("Invalid arguments!", 19);
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
              size_t messageSize = result.getMessage().length() + 1;
              char* message = new char[messageSize];
              result.getMessage().toCharArray(message,
                                              messageSize);
              m4SendChar(message, messageSize);
              delete[] message;
            }
            break;
          case FunctionRegistry::ExecuteResult::ArgumentError:
            m4SendChar("Error: Argument error", 22);
            break;
          case FunctionRegistry::ExecuteResult::FunctionNotFound:
            m4SendChar("Error: Function not found", 26);
            break;
        }
      }
      // m4SendData(getDebugBuffer());
    }
  }
};