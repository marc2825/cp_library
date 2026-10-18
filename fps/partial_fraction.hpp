#pragma once
#include <cstddef>
#include <set>
#include <stdexcept>
#include <vector>
#include "division.hpp"
#include "taylor_shift.hpp"

namespace cp {
struct PartialFractionDecomposition {
    std::vector<int> polynomial_part;
    // coefficients[i][j-1] / (x-roots[i])^j, j=1..multiplicities[i]。
    std::vector<std::vector<int>> coefficients;
};
// 分母は入力で完全に与えるQ=product_i (x-roots[i])^multiplicities[i]。
// 相異なる根、正の重複度、D=sum multiplicities <= Modが必要。
template <int Mod = 998244353, int Root = 3>
PartialFractionDecomposition partial_fraction_decomposition(const std::vector<int>& numerator,
    const std::vector<int>& roots, const std::vector<std::size_t>& multiplicities) {
    using F = fps_detail::Field<Mod>;
    if (roots.size() != multiplicities.size()) throw std::invalid_argument("pole array size mismatch");
    std::set<int> used; std::size_t degree = 0;
    for (std::size_t i = 0; i < roots.size(); ++i) {
        if (!multiplicities[i] || !used.insert(F::norm(roots[i])).second) throw std::invalid_argument("poles must be distinct with positive multiplicity");
        if (multiplicities[i] >= fps_max_size<Mod, Root>() - degree) throw std::length_error("partial fraction denominator too large");
        degree += multiplicities[i];
    }
    if (degree > static_cast<std::size_t>(Mod)) throw std::domain_error("partial fraction Taylor shifts require degree <= modulus");
    std::vector<int> denominator{1}; std::vector<std::vector<int>> factors;
    for (std::size_t i = 0; i < roots.size(); ++i) {
        std::vector<int> factor{1};
        for (std::size_t j = 0; j < multiplicities[i]; ++j) factor = fps_detail::multiply<Mod, Root>(factor, {F::sub(0, F::norm(roots[i])), 1});
        factors.push_back(factor); denominator = fps_detail::multiply<Mod, Root>(denominator, factor);
    }
    auto division = polynomial_divmod<Mod, Root>(numerator, denominator);
    PartialFractionDecomposition result{division.first, {}};
    for (std::size_t i = 0; i < roots.size(); ++i) {
        const auto other = polynomial_quotient<Mod, Root>(denominator, factors[i]);
        const auto shifted = taylor_shift<Mod, Root>(division.second, roots[i]);
        const auto local = taylor_shift<Mod, Root>(other, roots[i]);
        auto coefficients = fps_detail::product_prefix<Mod, Root>(shifted, fps_inv<Mod, Root>(local, multiplicities[i]), multiplicities[i]);
        std::reverse(coefficients.begin(), coefficients.end()); result.coefficients.push_back(coefficients);
    }
    return result;
}
} // namespace cp
