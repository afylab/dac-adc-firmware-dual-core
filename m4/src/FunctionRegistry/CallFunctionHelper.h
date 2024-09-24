// CallFunctionHelper.h
#pragma once
#include <vector>
#include <tuple>

// Helper function to cast and unpack arguments from the vector
template <typename Function, typename Tuple, size_t... Is>
auto callFunctionHelper(Function function, const std::vector<float>& args, std::index_sequence<Is...>) 
    -> decltype(function(static_cast<typename std::tuple_element<Is, Tuple>::type>(args[Is])...)) 
{
    return function(static_cast<typename std::tuple_element<Is, Tuple>::type>(args[Is])...);
}
