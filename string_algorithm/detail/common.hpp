#pragma once
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace cp { namespace string_algorithm_detail {
inline int checked_size(std::size_t size) {
    if (size > static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::length_error("string length exceeds int range");
    return static_cast<int>(size);
}
inline void range(int size, int left, int right) {
    if (left < 0 || right < left || right > size) throw std::out_of_range("string interval is outside [0,size]");
}
} } // namespace cp::string_algorithm_detail
