#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <vector>
#include "detail/common.hpp"

namespace cp {
// byte+1の多項式ハッシュ。二つの素数法でも衝突はあり、等値・LCPが誤る可能性がある。
class RollingHash {
public:
    using Hash = std::array<std::uint32_t, 2>;
    inline static constexpr Hash moduli{1000000007, 1000000009};
    inline static constexpr std::uint32_t default_base = 911382323;
private:
    std::uint32_t base_;
    std::vector<Hash> prefix_, powers_;
    Hash power(int length) const { return powers_.empty() ? Hash{1, 1} : powers_[length]; }
    void validate(Hash value) const {
        for (std::size_t j = 0; j < 2; ++j) if (value[j] >= moduli[j]) throw std::invalid_argument("hash residue is not normalized");
    }
    void compatible(const RollingHash& other) const {
        if (base_ != other.base_) throw std::invalid_argument("rolling hash bases differ");
    }
public:
    explicit RollingHash(std::string_view s = {}, std::uint32_t base = default_base) : base_(base) {
        const int n = string_algorithm_detail::checked_size(s.size());
        if (base < 257 || base >= moduli[0]) throw std::invalid_argument("rolling hash base must be in [257,1000000007)");
        prefix_.resize(static_cast<std::size_t>(n) + 1); powers_.resize(prefix_.size()); powers_[0] = {1, 1};
        for (int i = 0; i < n; ++i) for (std::size_t j = 0; j < 2; ++j) {
            powers_[i + 1][j] = static_cast<std::uint32_t>(std::uint64_t(powers_[i][j]) * base_ % moduli[j]);
            prefix_[i + 1][j] = static_cast<std::uint32_t>((std::uint64_t(prefix_[i][j]) * base_ + static_cast<unsigned char>(s[i]) + 1) % moduli[j]);
        }
    }
    int size() const { return prefix_.empty() ? 0 : static_cast<int>(prefix_.size() - 1); }
    std::uint32_t base() const { return base_; }
    Hash get(int left, int right) const {
        string_algorithm_detail::range(size(), left, right);
        if (left == right) return {0, 0};
        Hash result{}; const auto p = power(right - left);
        for (std::size_t j = 0; j < 2; ++j) result[j] = static_cast<std::uint32_t>((prefix_[right][j] + std::uint64_t(moduli[j]) - std::uint64_t(prefix_[left][j]) * p[j] % moduli[j]) % moduli[j]);
        return result;
    }
    Hash hash_string(std::string_view s) const {
        string_algorithm_detail::checked_size(s.size()); Hash result{};
        for (unsigned char byte : s) for (std::size_t j = 0; j < 2; ++j) result[j] = static_cast<std::uint32_t>((std::uint64_t(result[j]) * base_ + byte + 1) % moduli[j]);
        return result;
    }
    // raw Hashは同じbaseで計算し、right_length<=size()。長さ・base情報自体はHashに含まれない。
    Hash concat(Hash left, Hash right, int right_length) const {
        string_algorithm_detail::range(size(), 0, right_length); validate(left); validate(right);
        Hash result{}; const auto p = power(right_length);
        for (std::size_t j = 0; j < 2; ++j) result[j] = static_cast<std::uint32_t>((std::uint64_t(left[j]) * p[j] + right[j]) % moduli[j]);
        return result;
    }
    bool equal(const RollingHash& other, int left, int right, int other_left, int other_right) const {
        string_algorithm_detail::range(size(), left, right); string_algorithm_detail::range(other.size(), other_left, other_right); compatible(other);
        return right - left == other_right - other_left && get(left, right) == other.get(other_left, other_right);
    }
    int lcp(const RollingHash& other, int left, int right, int other_left, int other_right) const {
        string_algorithm_detail::range(size(), left, right); string_algorithm_detail::range(other.size(), other_left, other_right); compatible(other);
        int low = 0, high = std::min(right - left, other_right - other_left);
        while (low < high) {
            const int mid = low + (high - low) / 2 + 1;
            if (get(left, left + mid) == other.get(other_left, other_left + mid)) low = mid; else high = mid - 1;
        }
        return low;
    }
    int lcp(const RollingHash& other, int left, int other_left) const { return lcp(other, left, size(), other_left, other.size()); }
};
} // namespace cp
