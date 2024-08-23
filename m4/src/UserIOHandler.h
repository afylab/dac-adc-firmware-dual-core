#pragma once

#include <Arduino.h>

#include <vector>

#include "FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

#include "RPC.h"

struct UserIOHandler {

  static void setup() {
    RPC.begin();
    REGISTER_MEMBER_FUNCTION_0(nop, "NOP");
    REGISTER_MEMBER_FUNCTION_0(id, "*IDN?");
    REGISTER_MEMBER_FUNCTION_0(rdy, "*RDY?");
    REGISTER_MEMBER_FUNCTION_0(serialNumber, "SERIAL_NUMBER");
    
  }

  static OperationResult nop() { return OperationResult::Success("NOP");}
  static OperationResult id() { return OperationResult::Success("DAC-ADC_AD7734-AD5791");}
  static OperationResult rdy() { return OperationResult::Success("READY");}
  static OperationResult serialNumber() { return OperationResult::Success("DA20_16_08");}
  

  static std::vector<String> query_rpc() {
    char received = '\0';
    String inByte = "";
    std::vector<String> comm;
    while (received != '\r')  // Wait for carriage return
    {
      if (RPC.available()) {
        received = RPC.read();
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

  static bool isValidFloat(const String& str) {
    char* endPtr;
    strtod(str.c_str(), &endPtr);
    return endPtr != str.c_str() && *endPtr == '\0' && str.length() > 0;
  }

  static void handleUserIO() {
    std::vector<String> comm;
    if (RPC.available()) {
      comm = query_rpc();

      if (comm.size() > 0) {
        String command = comm[0];
        std::vector<float> args;

        for (size_t i = 1; i < comm.size(); ++i) {
          if (!isValidFloat(comm[i])) {
            RPC.write("Invalid arguments!");
            return;
          }
          args.push_back(comm[i].toFloat());
        }
        OperationResult result = OperationResult::Failure("Something went wrong!");
        FunctionRegistry::ExecuteResult executeResult =
            FunctionRegistry::execute(command, args, result);

        switch (executeResult) {
          case FunctionRegistry::ExecuteResult::Success:
            if (result.hasMessage()) {
              char* message = new char[result.getMessage().length() + 1];
              result.getMessage().toCharArray(message, result.getMessage().length() + 1);
              RPC.write(message);
              delete[] message;
            }
            break;
          case FunctionRegistry::ExecuteResult::ArgumentError:
            RPC.write("Error: Argument error");
            break;
          case FunctionRegistry::ExecuteResult::FunctionNotFound:
            RPC.write("Error: Function not found");
            break;
        }
      }
    }
  }
};