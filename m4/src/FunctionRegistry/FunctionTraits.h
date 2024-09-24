// FunctionTraits.h
#pragma once
#include <tuple>
#include <type_traits>

// Primary template for FunctionTraits (handles general callables)
template <typename T>
struct FunctionTraits;

// Specialization for function pointers
template <typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType(*)(Args...)> {
    using return_type = ReturnType;
    using args_tuple = std::tuple<Args...>;
    static constexpr size_t arity = sizeof...(Args);
};

// Specialization for member function pointers
template <typename ClassType, typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType(ClassType::*)(Args...)> {
    using return_type = ReturnType;
    using args_tuple = std::tuple<Args...>;
    static constexpr size_t arity = sizeof...(Args);
};

// Specialization for const member function pointers
template <typename ClassType, typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType(ClassType::*)(Args...) const> {
    using return_type = ReturnType;
    using args_tuple = std::tuple<Args...>;
    static constexpr size_t arity = sizeof...(Args);
};

// Specialization for functors (e.g., lambdas)
template <typename Functor>
struct FunctionTraits : public FunctionTraits<decltype(&Functor::operator())> {};
