#pragma once
#include "fft.hpp"
namespace cp {
// Prime-length DFT via the multiplicative group modulo n and a cyclic convolution.
template <class Real> void rader_fft(std::vector<std::complex<Real>> &a, bool inverse = false) {
    const auto n = a.size();
    if (n > 2000000000ULL || !convolution_detail::prime(static_cast<long long>(n)))
        throw std::invalid_argument("Rader FFT needs a prime length <= 2e9");
    for (const auto &x : a)
        if (!convolution_detail::finite(x))
            throw std::invalid_argument("FFT input must be finite");
    std::vector<std::size_t> factors;
    auto rem = n - 1;
    for (std::size_t p = 2; p <= rem / p; ++p)
        if (rem % p == 0) {
            factors.push_back(p);
            while (rem % p == 0)
                rem /= p;
        }
    if (rem > 1)
        factors.push_back(rem);
    std::size_t root = 1;
    for (;; ++root) {
        bool good = true;
        for (auto p : factors)
            if (convolution_detail::power(root, (n - 1) / p, n) == 1)
                good = false;
        if (good)
            break;
    }
    auto shape = convolution_detail::shape(n - 1, n - 1, std::vector<std::complex<Real>>().max_size());
    std::vector<std::size_t> index(n - 1);
    index[0] = 1;
    for (std::size_t i = 1; i < n - 1; ++i)
        index[i] = index[i - 1] * root % n;
    std::vector<std::complex<Real>> x(shape.second), y(shape.second), result(n);
    result[0] = a[0];
    const auto angle = (inverse ? -2 : 2) * std::acos(Real(-1)) / static_cast<Real>(n);
    for (std::size_t i = 0; i < n - 1; ++i) {
        x[i] = a[index[(n - 1 - i) % (n - 1)]];
        y[i] = std::polar(Real(1), angle * static_cast<Real>(index[i]));
        result[0] += a[index[i]];
    }
    fft(x);
    fft(y);
    for (std::size_t i = 0; i < x.size(); ++i)
        x[i] *= y[i];
    fft(x, true);
    for (std::size_t i = 0; i < n - 1; ++i) {
        result[index[i]] = a[0] + x[i];
        if (i + n - 1 < shape.first)
            result[index[i]] += x[i + n - 1];
    }
    for (auto &v : result) {
        if (inverse)
            v /= static_cast<Real>(n);
        if (!convolution_detail::finite(v))
            throw std::overflow_error("FFT result is not finite");
    }
    a.swap(result);
}
} // namespace cp
