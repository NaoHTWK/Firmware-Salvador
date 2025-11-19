#pragma once

#include <map>
#include <mutex>
#include <optional>

namespace htwk {
// Thread-safe MapStore for key-value pairs
template <typename Key, typename Value>
class MapStore {
public:
    MapStore() = default;

    void put(const Key& key, const Value& value) {
        std::unique_lock<std::mutex> lock(mtx);
        data[key] = value;
    }

    std::optional<Value> get(const Key& key) {
        std::unique_lock<std::mutex> lock(mtx);
        auto it = data.find(key);
        if (it != data.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    bool contains(const Key& key) {
        std::unique_lock<std::mutex> lock(mtx);
        return data.find(key) != data.end();
    }

    size_t size() {
        std::unique_lock<std::mutex> lock(mtx);
        return data.size();
    }

    std::map<Key, Value> get_all() {
        std::unique_lock<std::mutex> lock(mtx);
        return data;
    }

    void clear_last_n(size_t n) {
        std::unique_lock<std::mutex> lock(mtx);
        if (data.size() > n) {
            auto it = data.begin();
            std::advance(it, n);
            data.erase(data.begin(), it);
        }
    }

    void clear_before(const Key& key) {
        std::unique_lock<std::mutex> lock(mtx);
        data.erase(data.begin(), data.find(key));
    }

private:
    std::map<Key, Value> data;
    std::mutex mtx;
};
}  // namespace htwk