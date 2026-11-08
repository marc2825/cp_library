#pragma once
#include "fft.hpp"
namespace cp {
// Arbitrary length, same positive-sign convention as fft. O(n log n).
template <class Real> void bluestein_fft(std::vector<std::complex<Real>> &a, bool inverse = false) {
    const auto n = a.size();
    if (!n)
        return;
    for (const auto &x : a)
        if (!convolution_detail::finite(x))
            throw std::invalid_argument("FFT input must be finite");
    auto shape = convolution_detail::shape(n, n, std::vector<std::complex<Real>>().max_size());
    std::vector<std::complex<Real>> x(shape.second), y(shape.second), chirp(n);
    const Real angle = (inverse ? -1 : 1) * std::acos(Real(-1)) / static_cast<Real>(n);
    for (std::size_t j = 0; j < n; ++j) {
        const auto square = std::fmod(static_cast<long double>(j) * j, static_cast<long double>(n) * 2);
        chirp[j] = std::polar(Real(1), angle * static_cast<Real>(square));
        x[j] = a[j] * chirp[j];
        y[j] = std::conj(chirp[j]);
        if (j)
            y[shape.second - j] = y[j];
    }
    fft(x);
    fft(y);
    for (std::size_t i = 0; i < x.size(); ++i)
        x[i] *= y[i];
    fft(x, true);
    auto result = a;
    for (std::size_t i = 0; i < n; ++i) {
        result[i] = x[i] * chirp[i];
        if (inverse)
            result[i] /= static_cast<Real>(n);
        if (!convolution_detail::finite(result[i]))
            throw std::overflow_error("FFT result is not finite");
    }
    a.swap(result);
}
} // namespace cp
