#pragma once
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../fps/polynomial.hpp"

namespace cp {
// f(start * ratio^k), k=0..count-1。ratio=0も扱い、最初の点はstart。
template <int Mod = 998244353, int Root = 3>
std::vector<int> chirp_z(const std::vector<int>& f, int start, int ratio, std::size_t count) {
    using F = fps_detail::Field<Mod>;
    if (count > std::vector<int>().max_size()) throw std::length_error("too many Chirp-Z points");
    if (!count) return {};
    if (f.empty()) return std::vector<int>(count);
    start = F::norm(start); ratio = F::norm(ratio);
    if (!ratio) {
        std::vector<int> result(count, F::norm(f[0])); result[0] = polynomial_evaluate<Mod>(f, start); return result;
    }
    const auto shape = convolution_detail::shape(f.size(), count, ntt_max_size<Mod, Root>());
    // 積は |f| と |f|+count-1 の2列。確保前に容量を検査。
    convolution_detail::shape(f.size(), shape.first, ntt_max_size<Mod, Root>());
    std::vector<int> triangle(shape.first, 1), inverse_triangle(shape.first, 1);
    const int inverse = F::inv(ratio); int power = 1, inverse_power = 1;
    for (std::size_t i = 1; i < shape.first; ++i) {
        triangle[i] = F::mul(triangle[i - 1], power);
        inverse_triangle[i] = F::mul(inverse_triangle[i - 1], inverse_power);
        power = F::mul(power, ratio); inverse_power = F::mul(inverse_power, inverse);
    }
    std::vector<int> a(f.size()); power = 1;
    for (std::size_t i = 0; i < f.size(); ++i) {
        a[f.size() - 1 - i] = F::mul(F::mul(F::norm(f[i]), power), inverse_triangle[i]); power = F::mul(power, start);
    }
    const auto product = fps_detail::multiply<Mod, Root>(a, triangle);
    std::vector<int> result(count);
    for (std::size_t k = 0; k < count; ++k) result[k] = F::mul(product[f.size() - 1 + k], inverse_triangle[k]);
    return result;
}
} // namespace cp
