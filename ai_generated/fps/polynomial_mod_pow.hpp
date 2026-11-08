#pragma once
#include <cstdint>
#include <vector>
#include "division.hpp"

namespace cp {
// a(x)^exponent mod modulus(x)。法多項式は非零、定数法の結果は空。
template <int Mod = 998244353, int Root = 3>
std::vector<int> polynomial_mod_pow(const std::vector<int>& a, std::uint64_t exponent, const std::vector<int>& modulus) {
    const auto divisor = polynomial_trim<Mod>(modulus);
    auto base = polynomial_remainder<Mod, Root>(a, divisor);
    auto result = polynomial_remainder<Mod, Root>({1}, divisor);
    for (; exponent; exponent >>= 1) {
        if (exponent & 1) result = polynomial_remainder<Mod, Root>(fps_detail::multiply<Mod, Root>(result, base), divisor);
        if (exponent > 1) base = polynomial_remainder<Mod, Root>(fps_detail::multiply<Mod, Root>(base, base), divisor);
    }
    return result;
}
} // namespace cp
