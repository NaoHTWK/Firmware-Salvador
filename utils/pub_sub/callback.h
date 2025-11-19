#pragma once

#include <functional>
#include <tuple>
#include <type_traits>
#include <vector>

template <typename T>
struct function_traits;

template <typename R, typename... Args>
struct function_traits<R(Args...)> {
    using return_type = R;
    using args_tuple = std::tuple<Args...>;
    static constexpr size_t arity = sizeof...(Args);

    template <size_t N>
    using arg_type = std::tuple_element_t<N, args_tuple>;
};

template <typename T>
class Callback {
public:
    void registerCallback(std::function<T> callback) {
        callbacks.push_back(callback);
    }

    // Call all registered callbacks
    // For void return type: executes all callbacks in sequence
    // For non-void return type: returns a vector of all results
    template <typename... Args,
              typename = std::enable_if_t<std::is_invocable_r_v<
                      typename function_traits<T>::return_type, std::function<T>, Args...>>>
    auto operator()(Args&&... args)
            -> std::conditional_t<std::is_void_v<typename function_traits<T>::return_type>, void,
                                  std::vector<typename function_traits<T>::return_type>> {
        if constexpr (std::is_void_v<typename function_traits<T>::return_type>) {
            // For void return type, call all callbacks in sequence
            for (auto& callback : callbacks) {
                callback(std::forward<Args>(args)...);
            }
        } else {
            // For non-void return type, call all callbacks and collect all results
            std::vector<typename function_traits<T>::return_type> results;
            results.reserve(callbacks.size());

            for (auto& callback : callbacks) {
                results.push_back(callback(std::forward<Args>(args)...));
            }

            return results;
        }
    }

private:
    std::vector<std::function<T>> callbacks;
};
