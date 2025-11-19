#pragma once

#include <boost/serialization/access.hpp>
#include <optional>

// Add boost serialization for optional
namespace boost {
namespace serialization {

template <class Archive, typename T>
void serialize(Archive& ar, std::optional<T>& opt, const unsigned int version) {
    bool has_value = opt.has_value();
    ar& has_value;

    if (has_value) {
        if (!opt) {
            opt.emplace();
        }
        ar&* opt;
    } else {
        opt.reset();
    }
}

}  // namespace serialization
}  // namespace boost
