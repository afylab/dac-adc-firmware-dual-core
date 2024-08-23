#pragma once
#include "FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

#define REGISTER_MEMBER_FUNCTION_VECTOR(functionName, commandName) \
  FunctionRegistry::registerFunction(                              \
      commandName,                                                 \
      [](const std::vector<float>& args) -> OperationResult {      \
        return functionName(args);                                 \
      },                                                           \
      -1)

#define REGISTER_MEMBER_FUNCTION_0(functionName, commandName) \
  FunctionRegistry::registerFunction(                         \
      commandName,                                            \
      [](const std::vector<float>&) -> OperationResult {      \
        return functionName();                                \
      },                                                      \
      0)

#define REGISTER_MEMBER_FUNCTION_1(functionName, commandName) \
  FunctionRegistry::registerFunction(                         \
      commandName,                                            \
      [](const std::vector<float>& args) -> OperationResult { \
        return functionName(static_cast<float>(args[0]));     \
      },                                                      \
      1)

#define REGISTER_MEMBER_FUNCTION_2(functionName, commandName) \
  FunctionRegistry::registerFunction(                         \
      commandName,                                            \
      [](const std::vector<float>& args) -> OperationResult { \
        return functionName(static_cast<float>(args[0]),      \
                            static_cast<float>(args[1]));     \
      },                                                      \
      2)

#define REGISTER_MEMBER_FUNCTION_3(functionName, commandName) \
  FunctionRegistry::registerFunction(                         \
      commandName,                                            \
      [](const std::vector<float>& args) -> OperationResult { \
        return functionName(static_cast<float>(args[0]),      \
                            static_cast<float>(args[1]),      \
                            static_cast<float>(args[2]));     \
      },                                                      \
      3)

#define REGISTER_MEMBER_FUNCTION_4(functionName, commandName)          \
  FunctionRegistry::registerFunction(                                  \
      commandName,                                                     \
      [](const std::vector<float>& args) -> OperationResult {          \
        return functionName(                                           \
            static_cast<float>(args[0]), static_cast<float>(args[1]),  \
            static_cast<float>(args[2]), static_cast<float>(args[3])); \
      },                                                               \
      4)

#define REGISTER_MEMBER_FUNCTION_5(functionName, commandName)         \
  FunctionRegistry::registerFunction(                                 \
      commandName,                                                    \
      [](const std::vector<float>& args) -> OperationResult {         \
        return functionName(                                          \
            static_cast<float>(args[0]), static_cast<float>(args[1]), \
            static_cast<float>(args[2]), static_cast<float>(args[3]), \
            static_cast<float>(args[4]));                             \
      },                                                              \
      5)

#define REGISTER_MEMBER_FUNCTION_6(functionName, commandName)          \
  FunctionRegistry::registerFunction(                                  \
      commandName,                                                     \
      [](const std::vector<float>& args) -> OperationResult {          \
        return functionName(                                           \
            static_cast<float>(args[0]), static_cast<float>(args[1]),  \
            static_cast<float>(args[2]), static_cast<float>(args[3]),  \
            static_cast<float>(args[4]), static_cast<float>(args[5])); \
      },                                                               \
      6)

#define REGISTER_MEMBER_FUNCTION_7(functionName, commandName)         \
  FunctionRegistry::registerFunction(                                 \
      commandName,                                                    \
      [](const std::vector<float>& args) -> OperationResult {         \
        return functionName(                                          \
            static_cast<float>(args[0]), static_cast<float>(args[1]), \
            static_cast<float>(args[2]), static_cast<float>(args[3]), \
            static_cast<float>(args[4]), static_cast<float>(args[5]), \
            static_cast<float>(args[6]));                             \
      },                                                              \
      7)

#define REGISTER_MEMBER_FUNCTION_8(functionName, commandName)          \
  FunctionRegistry::registerFunction(                                  \
      commandName,                                                     \
      [](const std::vector<float>& args) -> OperationResult {          \
        return functionName(                                           \
            static_cast<float>(args[0]), static_cast<float>(args[1]),  \
            static_cast<float>(args[2]), static_cast<float>(args[3]),  \
            static_cast<float>(args[4]), static_cast<float>(args[5]),  \
            static_cast<float>(args[6]), static_cast<float>(args[7])); \
      },                                                               \
      8)
