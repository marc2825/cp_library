#pragma once
#include <complex>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "fft.hpp"

namespace cp {
// 実数・複素数の通常の畳み込み（巡回ではない）。自動0埋め、空は空。
template <class Real> std::vector<std::complex<Real>> convolution_fft(
        const std::vector<std::complex<Real>>& a, const std::vector<std::complex<Real>>& b) {
    static_assert(std::is_floating_point_v<Real>, "FFT convolution requires a floating point type");
    const auto sizes = convolution_detail::shape(a.size(), b.size(), a.max_size());
    if (!sizes.first) return {};
    std::vector<std::complex<Real>> x(a), y(b);
    x.resize(sizes.second); y.resize(sizes.second);
    fft(x); fft(y);
    for (std::size_t i = 0; i < x.size(); ++i) {
        x[i] *= y[i];
        if (!convolution_detail::finite(x[i])) throw std::overflow_error("FFT pointwise product is not finite");
    }
    fft(x, true); x.resize(sizes.first);
    return x;
}
template <class Real> std::vector<Real> convolution_fft(const std::vector<Real>& a, const std::vector<Real>& b) {
    static_assert(std::is_floating_point_v<Real>, "FFT convolution requires a floating point type");
    if (a.empty() || b.empty()) return {};
    const auto c = convolution_fft(std::vector<std::complex<Real>>(a.begin(), a.end()),
                                   std::vector<std::complex<Real>>(b.begin(), b.end()));
    std::vector<Real> result; result.reserve(c.size());
    for (const auto& x : c) result.push_back(x.real());
    return result;
}
} // namespace cp
