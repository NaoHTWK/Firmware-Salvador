#pragma once

#include <memory>
#include <string>

class Order {
private:
    std::string name;

public:
    explicit Order(std::string name) : name(std::move(name)) {}
    Order(const Order&) = delete;
    Order(Order&&) = delete;
    Order& operator=(const Order&) = delete;
    Order& operator=(Order&&) = delete;
    virtual ~Order();

    virtual const std::string& getClassName() const {
        return name;
    }

    virtual std::string to_string() const final {
        return name;
    }
};
