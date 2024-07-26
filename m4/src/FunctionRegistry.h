#pragma once

#include <Arduino.h>

#include <functional>
#include <type_traits>
#include <vector>

#include "Peripherals/OperationResult.h"

class FunctionRegistry {
 public:
  enum class ExecuteResult { Success, FunctionNotFound, ArgumentError };

 private:
  struct FunctionEntry {
    String name;
    std::function<OperationResult(const std::vector<float>&)> func;
    int argCount;
    bool is_void;

    FunctionEntry(const String& n,
                  std::function<OperationResult(const std::vector<float>&)> f,
                  int ac)
        : name(n), func(f), argCount(ac) {}
  };
  inline static std::vector<FunctionEntry> functions;


 public:
  template <typename Func>
  static void registerFunction(const String& name, Func&& func, int argCount) {
    String upper_name = name;
    upper_name.toUpperCase();


    functions.emplace_back(upper_name, func, argCount);
  }

  static ExecuteResult execute(const String& name, const std::vector<float>& args,
                        OperationResult& result) {
    String upper_name = name;
    upper_name.toUpperCase();
    for (const auto& entry : functions) {
      if (entry.name == upper_name) {
        if (entry.argCount > 0 && static_cast<int>(entry.argCount) != static_cast<int>(args.size())) {
          result = OperationResult::Failure("Argument count mismatch");
          return ExecuteResult::ArgumentError;
        }
        result = entry.func(args);
        return ExecuteResult::Success;
      }
    }
    result = OperationResult::Failure("Function not found");
    return ExecuteResult::FunctionNotFound;
  }
};