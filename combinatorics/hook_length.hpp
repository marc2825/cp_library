#pragma once
// 直線 Young 図形の hook-length / hook-content。説明: docs/combinatorics/hook_length.md
#include <climits>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>
#include "young_tableaux.hpp"
#include "../basic_utility/big_integer.hpp"
#include "../number_theory_basic/smallest_prime_factor.hpp"
#include "../number_theory_basic/detail/advanced_arithmetic.hpp"
namespace cp {
inline YoungTableau<std::size_t> young_hook_lengths(const YoungShape& shape, std::size_t max_cells = 1000000) {
    const auto columns = conjugate_young_shape(shape, max_cells);
    YoungTableau<std::size_t> hooks(shape.size());
    for (std::size_t r = 0; r < shape.size(); ++r) for (int c = 0; c < shape[r]; ++c)
        hooks[r].push_back(static_cast<std::size_t>(shape[r] - c) + static_cast<std::size_t>(columns[c]) - r - 1);
    return hooks;
}
namespace hook_length_detail {
inline std::vector<std::pair<int, int>> exponents(const YoungShape& shape, std::size_t max_cells) {
    const auto n = young_shape_size(shape, max_cells);
    if (n > static_cast<std::size_t>(INT_MAX)) throw std::length_error("hook sieve exceeds int range");
    SmallestPrimeFactor sieve(static_cast<int>(n));
    std::vector<int> exponent(n + 1);
    for (int p : sieve.primes()) for (std::size_t q = n / p; q; q /= p) exponent[p] += static_cast<int>(q);
    for (const auto& row : young_hook_lengths(shape, max_cells)) for (auto h : row)
        for (auto factor : sieve.factorize(static_cast<int>(h))) exponent[factor.first] -= factor.second;
    std::vector<std::pair<int, int>> result;
    for (int p : sieve.primes()) if (exponent[p]) result.emplace_back(p, exponent[p]);
    return result;
}
} // namespace hook_length_detail
inline BigInt standard_young_tableaux_count(const YoungShape& shape, std::size_t max_cells = 1000000) {
    BigInt result = 1;
    for (auto factor : hook_length_detail::exponents(shape, max_cells))
        result *= nt_detail::power(BigInt(factor.first), static_cast<unsigned>(factor.second));
    return result;
}
inline std::uint64_t standard_young_tableaux_count_mod(const YoungShape& shape, std::uint64_t modulus,
                                                       std::size_t max_cells = 1000000) {
    if (!modulus) throw std::invalid_argument("hook count modulus must be positive");
    const auto factors = hook_length_detail::exponents(shape, max_cells);
    if (modulus == 1) return 0;
    std::uint64_t result = 1;
    for (auto factor : factors) result = nt_detail::mul(result, nt_detail::pow(factor.first, factor.second, modulus), modulus);
    return result;
}
// ラベル 1..alphabet の半標準盤。任意精度整数で最後に厳密除算する。
inline BigInt semistandard_young_tableaux_count(const YoungShape& shape, std::uint64_t alphabet,
                                                std::size_t max_cells = 1000000) {
    young_shape_size(shape, max_cells);
    if (alphabet < shape.size()) return 0;
    BigInt numerator = 1, denominator = 1;
    const auto hooks = young_hook_lengths(shape, max_cells);
    for (std::size_t r = 0; r < shape.size(); ++r) for (int c = 0; c < shape[r]; ++c) {
        numerator *= BigInt(alphabet) + BigInt(c) - BigInt(r);
        denominator *= BigInt(hooks[r][c]);
    }
    return numerator / denominator;
}
} // namespace cp
