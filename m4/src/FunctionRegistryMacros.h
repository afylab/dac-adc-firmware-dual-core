#pragma once
#include "FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

#define REGISTER_MEMBER_FUNCTION_VECTOR(registry, functionName, commandName) \
  registry.registerFunction(                                                 \
      commandName,                                                           \
      [this](const std::vector<float>& args) -> OperationResult {            \
        return this->functionName(args);                                     \
      },                                                                     \
      -1)

#define REGISTER_MEMBER_FUNCTION_0(registry, functionName, commandName) \
  registry.registerFunction(                                            \
      commandName,                                                      \
      [this](const std::vector<float>&) -> OperationResult {            \
        return this->functionName();                                    \
      },                                                                \
      0)

#define REGISTER_MEMBER_FUNCTION_1(registry, functionName, commandName) \
  registry.registerFunction(                                            \
      commandName,                                                      \
      [this](const std::vector<float>& args) -> OperationResult {       \
        return this->functionName(static_cast<float>(args[0]));         \
      },                                                                \
      1)

#define REGISTER_MEMBER_FUNCTION_2(registry, functionName, commandName) \
  registry.registerFunction(                                            \
      commandName,                                                      \
      [this](const std::vector<float>& args) -> OperationResult {       \
        return this->functionName(static_cast<float>(args[0]),          \
                                  static_cast<float>(args[1]));         \
      },                                                                \
      2)

#define REGISTER_MEMBER_FUNCTION_3(registry, functionName, commandName) \
  registry.registerFunction(                                            \
      commandName,                                                      \
      [this](const std::vector<float>& args) -> OperationResult {       \
        return this->functionName(static_cast<float>(args[0]),          \
                                  static_cast<float>(args[1]),          \
                                  static_cast<float>(args[2]));         \
      },                                                                \
      3)

#define REGISTER_MEMBER_FUNCTION_4(registry, functionName, commandName) \
  registry.registerFunction(                                            \
      commandName,                                                      \
      [this](const std::vector<float>& args) -> OperationResult {       \
        return this->functionName(                                      \
            static_cast<float>(args[0]), static_cast<float>(args[1]),   \
            static_cast<float>(args[2]), static_cast<float>(args[3]));  \
      },                                                                \
      4)

#define REGISTER_MEMBER_FUNCTION_5(registry, functionName, commandName) \
  registry.registerFunction(                                            \
      commandName,                                                      \
      [this](const std::vector<float>& args) -> OperationResult {       \
        return this->functionName(                                      \
            static_cast<float>(args[0]), static_cast<float>(args[1]),   \
            static_cast<float>(args[2]), static_cast<float>(args[3]),   \
            static_cast<float>(args[4]));                               \
      },                                                                \
      5)

#define REGISTER_MEMBER_FUNCTION_6(registry, functionName, commandName) \
  registry.registerFunction(                                            \
      commandName,                                                      \
      [this](const std::vector<float>& args) -> OperationResult {       \
        return this->functionName(                                      \
            static_cast<float>(args[0]), static_cast<float>(args[1]),   \
            static_cast<float>(args[2]), static_cast<float>(args[3]),   \
            static_cast<float>(args[4]), static_cast<float>(args[5]));  \
      },                                                                \
      6)

#define REGISTER_MEMBER_FUNCTION_7(registry, functionName, commandName) \
  registry.registerFunction(                                            \
      commandName,                                                      \
      [this](const std::vector<float>& args) -> OperationResult {       \
        return this->functionName(                                      \
            static_cast<float>(args[0]), static_cast<float>(args[1]),   \
            static_cast<float>(args[2]), static_cast<float>(args[3]),   \
            static_cast<float>(args[4]), static_cast<float>(args[5]),   \
            static_cast<float>(args[6]));                               \
      },                                                                \
      7)
