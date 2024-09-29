// RegisterFunctions.h
#pragma once
#include <cassert>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "CallFunctionHelper.h"
#include "FunctionRegistry/FunctionRegistry.h"
#include "FunctionTraits.h"
#include "Peripherals/OperationResult.h"

// Template function to register functions with automatically deduced number of
// arguments
template <typename Function>
void registerMemberFunction(Function function, const String& commandName) {
  using Traits = FunctionTraits<Function>;
  constexpr size_t argCountSizeT = Traits::arity;
  constexpr int argCount = static_cast<int>(
      argCountSizeT);  // Ensure it matches FunctionRegistry's expected type

  using ArgsTuple = typename Traits::args_tuple;

  // Create a lambda that matches the expected signature for FunctionRegistry
  auto wrapper = [function](const std::vector<float>& args) -> OperationResult {
    // Ensure the number of arguments matches
    assert(args.size() == Traits::arity &&
           "Incorrect number of arguments provided.");

    // Call the helper to invoke the function with unpacked and casted arguments
    return callFunctionHelper<Function, ArgsTuple>(
        function, args, std::make_index_sequence<Traits::arity>{});
  };

  // Register the function with FunctionRegistry
  FunctionRegistry::registerFunction(
      commandName, wrapper,
      argCount  // Ensure this matches the expected type (int)
  );
}

// Template function to register functions that accept a vector of arguments
// directly
template <typename Function>
void registerMemberFunctionVector(Function function,
                                  const String& commandName) {
  FunctionRegistry::registerFunction(commandName, function,
                                     -1  // Use -1 or another sentinel value to
                                         // indicate variable argument count
  );
}
