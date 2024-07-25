#pragma once

#include <Arduino.h>

class OperationResult {
public:
    enum class Status {
        Success,
        Failure
    };

    static OperationResult Success(const String& message) {
        return OperationResult(Status::Success, message);
    }

    static OperationResult Success() {
        return OperationResult(Status::Success, "");
    }

    static OperationResult Failure(const String& message) {
        return OperationResult(Status::Failure, message);
    }

    static OperationResult Failure() {
        return OperationResult(Status::Failure, String("Something went wrong!"));
    }

    // Check if the operation was successful
    bool isSuccess() const {
        return status == Status::Success;
    }

    bool hasMessage() const {
        return message.length() > 0;
    }

    // Get the error message
    String getMessage() const {
        return message;
    }

private:
    Status status;
    String message;

    // Private constructors
    explicit OperationResult(Status s) : status(s) {}
    OperationResult(Status s, const String& message) : status(s), message(message) {}
};