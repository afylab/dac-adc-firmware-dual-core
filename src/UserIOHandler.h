#pragma once

#include <Arduino.h>

#include <vector>

#include "FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

struct UserIOHandler {
  FunctionRegistry& registry;

  UserIOHandler(FunctionRegistry& r) : registry(r) {}

  void setup() {
    Serial.begin(115200);
    REGISTER_MEMBER_FUNCTION_0(registry, nop, "NOP");
  }

  OperationResult nop() { return OperationResult::Success("NOP");}

  std::vector<String> query_serial() {
    char received = '\0';
    String inByte = "";
    std::vector<String> comm;
    while (received != '\r')  // Wait for carriage return
    {
      if (Serial.available()) {
        received = Serial.read();
        if (received == '\n' || received == ' ') {
        } else if (received == ',' || received == '\r') {
          comm.push_back(inByte);  // Adds string to vector of command arguments
          inByte = "";             // Resets to a null string for the next word
        } else {
          inByte += received;  // Adds newest char to end of string
        }
      }
    }
    return comm;
  }

  bool isValidFloat(const String& str) {
    char* endPtr;
    strtod(str.c_str(), &endPtr);
    return endPtr != str.c_str() && *endPtr == '\0' && str.length() > 0;
  }

  void handleUserIO() {
    std::vector<String> comm;
    if (Serial.available()) {
      comm = query_serial();

      if (comm.size() > 0) {
        String command = comm[0];
        std::vector<float> args;

        for (size_t i = 1; i < comm.size(); ++i) {
          if (!isValidFloat(comm[i])) {
            Serial.println("Invalid arguments!");
            return;
          }
          args.push_back(comm[i].toFloat());
        }
        OperationResult result = OperationResult::Failure("Something went wrong!");
        FunctionRegistry::ExecuteResult executeResult =
            registry.execute(command, args, result);

        switch (executeResult) {
          case FunctionRegistry::ExecuteResult::Success:
            if (result.hasMessage()) {
              Serial.println(result.getMessage());
            }
            break;
          case FunctionRegistry::ExecuteResult::ArgumentError:
            Serial.println(F("Error: Argument error"));
            break;
          case FunctionRegistry::ExecuteResult::FunctionNotFound:
            Serial.println(F("Error: Function not found"));
            break;
        }
      }
    }
  }
};