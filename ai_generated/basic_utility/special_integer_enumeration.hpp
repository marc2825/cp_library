#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cp {
namespace detail {
inline std::optional<std::uint64_t> mirror_decimal_prefix(std::uint64_t prefix, bool odd) {
    std::uint64_t value = prefix, tail = odd ? prefix / 10 : prefix;
    while (tail) {
        const auto digit = tail % 10;
        if (value > (std::numeric_limits<std::uint64_t>::max() - digit) / 10) return std::nullopt;
        value = value * 10 + digit; tail /= 10;
    }
    return value;
}
} // namespace detail

// Ascending decimal palindromes in [first,last), including 0. Callback false stops.
template<class Visit> bool enumerate_palindromic_integers(std::uint64_t first, std::uint64_t last, Visit visit) {
    if (first > last) throw std::invalid_argument("reversed integer range");
    if (first == last) return true;
    std::uint64_t power = 1;
    for (int length = 1; length <= 20; ++length) {
        if (length % 2 == 1) power *= 10;
        std::uint64_t lo = length == 1 ? 0 : power / 10, hi = power;
        // Mirroring is increasing, so a narrow high range does not scan smaller values.
        while (lo < hi) {
            const auto mid = lo + (hi - lo) / 2;
            const auto value = detail::mirror_decimal_prefix(mid, length % 2 != 0);
            if (!value || *value >= first) hi = mid; else lo = mid + 1;
        }
        for (auto prefix = lo; prefix < power; ++prefix) {
            const auto value = detail::mirror_decimal_prefix(prefix, length % 2 != 0);
            if (!value || *value >= last) break;
            if (!visit(*value)) return false;
        }
    }
    return true;
}

// Ascending integers whose canonical decimal representation uses only digits.
// 0 is emitted only when digit 0 is allowed; leading zeros are never generated.
template<class Visit> bool enumerate_digit_restricted_integers(
        std::uint64_t first, std::uint64_t last, std::vector<int> digits, Visit visit) {
    if (first > last) throw std::invalid_argument("reversed integer range");
    for (int d : digits) if (d < 0 || d > 9) throw std::invalid_argument("invalid decimal digit");
    std::sort(digits.begin(), digits.end());
    digits.erase(std::unique(digits.begin(), digits.end()), digits.end());
    if (first == last || digits.empty()) return true;
    if (first == 0 && digits.front() == 0 && !visit(std::uint64_t(0))) return false;
    const auto upper = last - 1;
    std::array<std::uint64_t, 20> powers{}; powers[0] = 1;
    for (int i = 1; i < 20; ++i) powers[i] = powers[i - 1] * 10;
    const auto walk = [&](auto&& self, std::uint64_t prefix, int remaining) -> bool {
        // The numerical interval of this prefix must intersect [first,upper].
        if (prefix < first / powers[remaining] || prefix > upper / powers[remaining]) return true;
        if (remaining == 0) return visit(prefix);
        for (int digit : digits) {
            const auto d = static_cast<std::uint64_t>(digit);
            if (d > upper || prefix > (upper - d) / 10) break;
            if (!self(self, prefix * 10 + d, remaining - 1)) return false;
        }
        return true;
    };
    for (int length = 1; length <= 20 && powers[length - 1] <= upper; ++length)
        for (int digit : digits)
            if (digit != 0 && !walk(walk, static_cast<std::uint64_t>(digit), length - 1)) return false;
    return true;
}

// Multiplicative closure of factors >= 2 in [1,last), ascending and duplicate-free.
// Empty factors yield only 1. Factors need not be prime or pairwise coprime.
template<class Visit> bool enumerate_generated_products(
        std::uint64_t last, std::vector<std::uint64_t> factors, Visit visit) {
    for (auto f : factors) if (f < 2) throw std::invalid_argument("product factor must be >= 2");
    std::sort(factors.begin(), factors.end());
    factors.erase(std::unique(factors.begin(), factors.end()), factors.end());
    if (last <= 1) return true;
    if (!visit(std::uint64_t(1))) return false;
    factors.erase(std::lower_bound(factors.begin(), factors.end(), last), factors.end());
    if (factors.empty()) return true;
    std::vector<std::uint64_t> sequence{1}, next = factors;
    std::vector<std::size_t> index(factors.size(), 0);
    for (;;) {
        const auto value = *std::min_element(next.begin(), next.end());
        if (value == last) return true;
        sequence.push_back(value);
        if (!visit(value)) return false;
        for (std::size_t i = 0; i < factors.size(); ++i) if (next[i] == value) {
            const auto x = sequence[++index[i]];
            next[i] = x > (last - 1) / factors[i] ? last : x * factors[i];
        }
    }
}
} // namespace cp
