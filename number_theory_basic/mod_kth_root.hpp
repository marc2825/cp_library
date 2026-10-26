#pragma once
#include <algorithm>
#include <cstdint>
#include <numeric>
#include <vector>
#include "discrete_log_table.hpp"
#include "primitive_root.hpp"

namespace cp {
// 非零解 first*ratio^j (0<=j<count)、および zero。列挙せず保持できる。
struct ModKthRoots {
    std::uint64_t modulus, first = 0, ratio = 1, count = 0;
    bool zero = false;
    std::vector<std::uint64_t> enumerate(std::uint64_t max_roots = 1000000) const {
        if (count > max_roots || (zero && count == max_roots)) throw std::length_error("root enumeration budget");
        std::vector<std::uint64_t> result;
        if (zero) result.push_back(0);
        auto x = first;
        for (std::uint64_t j = 0; j < count; ++j) { result.push_back(x); x = nt_detail::mul(x, ratio, modulus); }
        std::sort(result.begin(), result.end());
        return result;
    }
};
// Prime modulus, uint64 full range. x^0=1 including x=0. BSGS resource cap is explicit.
inline ModKthRoots mod_kth_roots(std::uint64_t a, std::uint64_t k, std::uint64_t prime,
                               std::uint64_t max_baby_steps = 2000000) {
    if (!is_prime_64(prime)) throw std::invalid_argument("kth roots require a prime modulus");
    a %= prime;
    ModKthRoots result{prime};
    if (k == 0) {
        if (a == 1) { result.zero = true; result.first = 1; result.ratio = primitive_root(prime); result.count = prime - 1; }
        return result;
    }
    if (a == 0) { result.zero = true; return result; }
    const auto generator = primitive_root(prime), order = prime - 1;
    const auto log = DiscreteLogTable(generator, prime, max_baby_steps).query(a);
    const auto g = std::gcd(k, order);
    if (*log % g != 0) return result;
    const auto period = order / g;
    const auto exponent = nt_detail::mul(*log / g % period, nt_detail::inverse(k / g % period, period), period);
    result.first = nt_detail::pow(generator, exponent, prime);
    result.ratio = nt_detail::pow(generator, period, prime);
    result.count = g;
    return result;
}
} // namespace cp
