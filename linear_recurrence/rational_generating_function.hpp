#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "../fps/polynomial_gcd.hpp"
#include "../fps/bostan_mori.hpp"
#include "coefficient_extraction.hpp"

namespace cp {
// x=0で正則な有理母関数。gcdで約分し分母定数を1に正規化する。
template <int Mod = 998244353, int Root = 3> class RationalGeneratingFunction {
    using F = fps_detail::Field<Mod>;
    std::vector<int> p_, q_;
public:
    RationalGeneratingFunction(const std::vector<int>& p = {}, const std::vector<int>& q = {1})
        : p_(polynomial_trim<Mod>(p)), q_(polynomial_trim<Mod>(q)) {
        if (q_.empty() || !q_[0]) throw std::domain_error("rational generating function needs nonzero denominator constant");
        const auto common = polynomial_gcd<Mod, Root>(p_, q_);
        p_ = polynomial_quotient<Mod, Root>(p_, common); q_ = polynomial_quotient<Mod, Root>(q_, common);
        const int scale = F::inv(q_[0]);
        for (auto& x : p_) x = F::mul(x, scale);
        for (auto& x : q_) x = F::mul(x, scale);
    }
    const std::vector<int>& numerator() const { return p_; }
    const std::vector<int>& denominator() const { return q_; }
    int coefficient(std::uint64_t index) const { return bostan_mori<Mod, Root>(p_, q_, index); }
    std::vector<int> prefix(std::size_t count) const { return generating_function_coefficients<Mod, Root>(p_, q_, count); }
    friend RationalGeneratingFunction operator+(const RationalGeneratingFunction& a, const RationalGeneratingFunction& b) {
        return {polynomial_add<Mod>(polynomial_multiply<Mod, Root>(a.p_, b.q_), polynomial_multiply<Mod, Root>(b.p_, a.q_)), polynomial_multiply<Mod, Root>(a.q_, b.q_)};
    }
    friend RationalGeneratingFunction operator*(const RationalGeneratingFunction& a, const RationalGeneratingFunction& b) {
        return {polynomial_multiply<Mod, Root>(a.p_, b.p_), polynomial_multiply<Mod, Root>(a.q_, b.q_)};
    }
};
} // namespace cp
