#pragma once
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "safe_integer.hpp"

namespace cp {
template<class T> struct MaximumSubarrayResult { T sum; std::size_t left, right; };
// Integral sums are checked. T may also be an exact ordered additive type (BigInt).
// Ties: lexicographically smallest (left,right). Empty input -> nullopt unless allowed.
template<class T>
std::optional<MaximumSubarrayResult<T>> maximum_subarray(const std::vector<T>& a, bool allow_empty = false) {
    std::optional<MaximumSubarrayResult<T>> best;
    if (allow_empty) best = MaximumSubarrayResult<T>{T(0), 0, 0};
    T sum = T(0);
    std::size_t left = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (sum < T(0)) { sum = T(0); left = i; }
        if constexpr (detail::is_integer_v<T>) {
            auto next = checked_add(sum, a[i]);
            if (!next) throw std::overflow_error("maximum subarray sum overflow");
            sum = *next;
        } else sum += a[i];
        if (!best || best->sum < sum ||
            (!(sum < best->sum) && std::make_pair(left, i + 1) < std::make_pair(best->left, best->right)))
            best = MaximumSubarrayResult<T>{sum, left, i + 1};
    }
    return best;
}
} // namespace cp
