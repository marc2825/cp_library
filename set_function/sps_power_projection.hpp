#pragma once
#include "fast_subset_convolution.hpp"
namespace cp {
// result[k]=sum_S weights[S]*(f^{*k})[S], disjoint subset product.
// Separate the nilpotent part; O(b^3 2^b + b*count), all prime characteristics supported.
template <int Mod = 998244353>
std::vector<int> sps_power_projection(const std::vector<int> &weights, const std::vector<int> &f,
                                      std::size_t count) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    set_function_detail::validate_size(f.size());
    if (weights.size() != f.size())
        throw std::invalid_argument("SPS projection sizes differ");
    std::size_t bits = 0;
    for (auto s = f.size(); s > 1; s /= 2)
        ++bits;
    auto nil = f;
    int constant = F::norm(nil[0]);
    nil[0] = 0;
    auto degree = std::min(bits, count ? count - 1 : 0);
    std::vector<int> moments(degree + 1), power(f.size());
    power[0] = 1;
    for (std::size_t j = 0; j <= degree; ++j) {
        for (std::size_t s = 0; s < f.size(); ++s)
            moments[j] = F::add(moments[j], F::mul(F::norm(weights[s]), power[s]));
        if (j < degree)
            power = subset_convolution_mod<Mod>(power, nil);
    }
    std::vector<int> out(count), row(degree + 1);
    row[0] = 1;
    for (std::size_t k = 0; k < count; ++k) {
        for (std::size_t j = 0; j <= std::min(k, degree); ++j)
            out[k] = F::add(out[k], F::mul(row[j], moments[j]));
        for (std::size_t j = degree; j > 0; --j)
            row[j] = F::add(F::mul(constant, row[j]), row[j - 1]);
        row[0] = F::mul(constant, row[0]);
    }
    return out;
}
} // namespace cp
