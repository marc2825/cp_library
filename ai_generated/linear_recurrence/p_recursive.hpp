#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "detail/common.hpp"

namespace cp {
// sum_{j=0}^r p[j](t)*a[t+j]=0。p[j]は昇冪係数。
// initialは正確にr項、r>=1。先頭からcount項を逐次生成する。
template <int Mod = 998244353> std::vector<int> p_recursive_prefix(
    const std::vector<int>& initial, const std::vector<std::vector<int>>& p, std::size_t count) {
    static_assert(linear_recurrence_detail::prime(Mod), "P-recursive division requires a prime modulus");
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (p.size() < 2 || initial.size() != p.size() - 1) throw std::invalid_argument("P-recursive order and initial size differ");
    if (count > std::vector<int>().max_size()) throw std::length_error("P-recursive prefix too large");
    const auto order = initial.size();
    std::vector<int> result(count);
    for (std::size_t i = 0; i < std::min(count, order); ++i) result[i] = F::norm(initial[i]);
    auto evaluate = [&](const std::vector<int>& polynomial, std::size_t index) {
        int value = 0, x = static_cast<int>(index % Mod);
        for (std::size_t j = polynomial.size(); j-- > 0;) value = F::add(F::mul(value, x), F::norm(polynomial[j]));
        return value;
    };
    for (std::size_t i = order; i < count; ++i) {
        const auto t = i - order; int sum = 0;
        for (std::size_t j = 0; j < order; ++j) sum = F::add(sum, F::mul(evaluate(p[j], t), result[t + j]));
        const int leading = evaluate(p.back(), t);
        if (!leading) throw std::domain_error("P-recursive leading coefficient vanishes at a generated step");
        result[i] = F::mul(F::sub(0, sum), F::inverse(leading));
    }
    return result;
}
} // namespace cp
