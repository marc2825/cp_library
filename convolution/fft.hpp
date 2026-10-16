#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "detail/common.hpp"

namespace cp {
namespace convolution_detail {
template <class Real> bool finite(const std::complex<Real>& x) { return std::isfinite(x.real()) && std::isfinite(x.imag()); }
} // namespace convolution_detail
// 順変換はΣ a[j] exp(+2πijk/n)、逆変換は負の角と1/n。自然順の全n成分。
// 空、1要素、2冪長に対応。有限値のみ。丸め誤差を含む近似計算。
template <class Real> void fft(std::vector<std::complex<Real>>& a, bool inverse = false) {
    static_assert(std::is_floating_point_v<Real>, "FFT requires a floating point type");
    const auto n = a.size();
    if (!n) return;
    if (!convolution_detail::power_of_two(n)) throw std::invalid_argument("FFT length must be a power of two");
    for (const auto& x : a) if (!convolution_detail::finite(x)) throw std::invalid_argument("FFT input must be finite");
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        auto bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    const auto pi = std::acos(Real(-1));
    for (std::size_t len = 2; len <= n; len *= 2) {
        const auto root = std::polar(Real(1), (inverse ? -2 : 2) * pi / static_cast<Real>(len));
        for (std::size_t first = 0; first < n; first += len) {
            std::complex<Real> w(1);
            for (std::size_t j = 0; j < len / 2; ++j) {
                const auto u = a[first + j], v = a[first + j + len / 2] * w;
                const auto left = u + v, right = u - v;
                if (!convolution_detail::finite(left) || !convolution_detail::finite(right))
                    throw std::overflow_error("FFT intermediate value is not finite");
                a[first + j] = left; a[first + j + len / 2] = right;
                w *= root;
            }
        }
        if (len == n) break;
    }
    if (inverse) for (auto& x : a) x /= static_cast<Real>(n);
}
// 実数入力の全スペクトル。半分へ圧縮せず、長さ条件もfftと同じ。
template <class Real> std::vector<std::complex<Real>> fft_real(const std::vector<Real>& a) {
    static_assert(std::is_floating_point_v<Real>, "real FFT requires a floating point type");
    std::vector<std::complex<Real>> result(a.begin(), a.end());
    fft(result);
    return result;
}
} // namespace cp
