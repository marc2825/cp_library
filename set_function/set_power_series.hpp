#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "fast_subset_convolution.hpp"

namespace cp {
namespace set_function_detail {
template <int Mod> std::vector<int> sps_function(const std::vector<int>& f, int operation) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    if (f.size() == 1) {
        if (operation == 0) return {F::inverse(F::norm(f[0]))};
        return {operation == 1 ? 1 : 0};
    }
    const auto half = f.size() / 2;
    const std::vector<int> low(f.begin(), f.begin() + half), high(f.begin() + half, f.end());
    auto g = sps_function<Mod>(low, operation);
    std::vector<int> upper;
    if (operation == 1) upper = subset_convolution_mod<Mod>(g, high);
    else {
        const auto inverse = operation == 0 ? g : sps_function<Mod>(low, 0);
        upper = subset_convolution_mod<Mod>(inverse, high);
        if (operation == 0) {
            upper = subset_convolution_mod<Mod>(inverse, upper);
            for (auto& value : upper) value = F::sub(0, value);
        }
    }
    g.insert(g.end(), upper.begin(), upper.end()); return g;
}
} // namespace set_function_detail
// SPSの積はdisjoint subset convolution。正の2冪長、素数法。
template <int Mod = 998244353> std::vector<int> sps_inv(const std::vector<int>& f) {
    set_function_detail::validate_size(f.size());
    if (!linear_recurrence_detail::Arithmetic<Mod>::norm(f[0])) throw std::domain_error("SPS inverse needs nonzero empty-set coefficient");
    return set_function_detail::sps_function<Mod>(f, 0);
}
template <int Mod = 998244353> std::vector<int> sps_exp(const std::vector<int>& f) {
    set_function_detail::validate_size(f.size());
    if (linear_recurrence_detail::Arithmetic<Mod>::norm(f[0])) throw std::domain_error("SPS exp needs zero empty-set coefficient");
    return set_function_detail::sps_function<Mod>(f, 1);
}
template <int Mod = 998244353> std::vector<int> sps_log(const std::vector<int>& f) {
    set_function_detail::validate_size(f.size());
    if (linear_recurrence_detail::Arithmetic<Mod>::norm(f[0]) != 1) throw std::domain_error("SPS log needs empty-set coefficient one");
    return set_function_detail::sps_function<Mod>(f, 2);
}
} // namespace cp
