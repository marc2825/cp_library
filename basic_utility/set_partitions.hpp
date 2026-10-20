#pragma once
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>
#include "big_integer.hpp"

namespace cp {
inline BigInt set_partition_count(int n) {
    if (n < 0) throw std::invalid_argument("negative set size");
    std::vector<BigInt> stirling(static_cast<std::size_t>(n) + 1); stirling[0] = 1;
    for (std::size_t size = 1; size <= static_cast<std::size_t>(n); ++size) {
        for (std::size_t blocks = size; blocks; --blocks) stirling[blocks] = stirling[blocks] * BigInt(blocks) + stirling[blocks - 1];
        stirling[0] = 0;
    }
    BigInt result = 0; for (const auto& x : stirling) result += x; return result;
}
inline std::vector<std::vector<int>> partition_blocks(const std::vector<int>& rgs) {
    if (rgs.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::length_error("partition too large");
    std::vector<std::vector<int>> blocks;
    for (std::size_t i = 0; i < rgs.size(); ++i) {
        int block = rgs[i];
        if (block < 0 || static_cast<std::size_t>(block) > blocks.size()) throw std::invalid_argument("invalid restricted-growth string");
        if (static_cast<std::size_t>(block) == blocks.size()) blocks.emplace_back();
        blocks[block].push_back(static_cast<int>(i));
    }
    return blocks;
}
// Lexicographic restricted-growth strings. Empty set has one partition.
// Return false from visit to stop. Callback receives a temporary view. O(n) delay/space.
template<class Visit> bool enumerate_set_partitions(int n, Visit visit) {
    if (n < 0) throw std::invalid_argument("negative set size");
    std::vector<int> rgs(n), prefix(n);
    for (;;) {
        if (!visit(static_cast<const std::vector<int>&>(rgs))) return false;
        int pos = n - 1;
        while (pos > 0 && rgs[pos] == prefix[pos - 1] + 1) --pos;
        if (pos <= 0) return true;
        ++rgs[pos]; prefix[pos] = std::max(prefix[pos - 1], rgs[pos]);
        for (int i = pos + 1; i < n; ++i) { rgs[i] = 0; prefix[i] = prefix[pos]; }
    }
}
} // namespace cp
