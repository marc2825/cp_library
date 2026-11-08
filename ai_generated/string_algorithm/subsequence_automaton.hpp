#pragma once
#include "detail/common.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace cp {
// States are consumed prefix lengths. Dense next-occurrence table for all 256 bytes.
class SubsequenceAutomaton {
    std::vector<std::array<int, 256>> next_;

  public:
    explicit SubsequenceAutomaton(std::string_view text = {}) {
        const int n = string_algorithm_detail::checked_size(text.size());
        next_.resize(text.size() + 1);
        next_.back().fill(-1);
        for (int i = n; i-- > 0;) {
            next_[i] = next_[i + 1];
            next_[i][static_cast<unsigned char>(text[i])] = i + 1;
        }
    }
    int size() const { return static_cast<int>(next_.size() - 1); }
    int next(int state, unsigned char byte) const {
        string_algorithm_detail::range(size(), state, state);
        return next_[state][byte];
    }
    std::optional<std::vector<int>> match(std::string_view pattern, int start = 0) const {
        string_algorithm_detail::range(size(), start, start);
        std::vector<int> positions;
        for (unsigned char c : pattern) {
            start = next_[start][c];
            if (start < 0)
                return std::nullopt;
            positions.push_back(start - 1);
        }
        return positions;
    }
    bool contains(std::string_view pattern, int start = 0) const { return bool(match(pattern, start)); }
};
// Includes the empty subsequence unless requested otherwise. Any positive uint64 modulus.
inline std::uint64_t distinct_subsequences(std::string_view text, std::uint64_t modulus,
                                           bool include_empty = true) {
    if (!modulus)
        throw std::invalid_argument("subsequence modulus must be positive");
    std::array<std::uint64_t, 256> previous{};
    std::uint64_t total = 1 % modulus;
    for (unsigned char c : text) {
        const auto doubled = total >= modulus - total ? total - (modulus - total) : total + total;
        const auto next = doubled >= previous[c] ? doubled - previous[c] : modulus - (previous[c] - doubled);
        previous[c] = total;
        total = next;
    }
    return include_empty ? total : (total ? total - 1 : modulus - 1);
}
} // namespace cp
