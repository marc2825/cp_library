#pragma once
#include <stdexcept>
#include <vector>
#include "big_integer.hpp"

namespace cp {
inline BigInt integer_partition_count(int n) {
    if (n < 0) throw std::invalid_argument("negative integer partition target");
    std::vector<BigInt> dp(static_cast<std::size_t>(n) + 1); dp[0] = 1;
    for (std::size_t part = 1; part <= static_cast<std::size_t>(n); ++part)
        for (std::size_t total = part; total <= static_cast<std::size_t>(n); ++total) dp[total] += dp[total - part];
    return dp[n];
}
// Nonincreasing parts, descending lexicographic order. n=0 emits the empty partition.
// Callback sees a temporary view; return false to stop. No recursion, O(n) delay/space.
template<class Visit> bool enumerate_integer_partitions(int n, Visit visit) {
    if (n < 0) throw std::invalid_argument("negative integer partition target");
    std::vector<int> parts; if (n) parts.push_back(n);
    for (;;) {
        if (!visit(static_cast<const std::vector<int>&>(parts))) return false;
        int remainder = 0;
        while (!parts.empty() && parts.back() == 1) { parts.pop_back(); ++remainder; }
        if (parts.empty()) return true;
        --parts.back(); ++remainder;
        const int cap = parts.back();
        while (remainder > cap) { parts.push_back(cap); remainder -= cap; }
        parts.push_back(remainder);
    }
}
} // namespace cp
