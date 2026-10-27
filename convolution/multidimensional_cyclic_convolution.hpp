#pragma once
#include "multidimensional_convolution.hpp"
namespace cp {
// Tensor on Z/s[0] x ... x Z/s[d-1], row-major. Arbitrary positive axis lengths.
template <int Mod = 998244353, int Root = 3>
std::vector<int> multidimensional_cyclic_convolution(const std::vector<int> &a, const std::vector<int> &b,
                                                     const std::vector<std::size_t> &shape) {
    auto full = multidimensional_convolution<Mod, Root>(a, shape, b, shape);
    std::vector<int> out(a.size());
    for (std::size_t i = 0; i < full.coefficients.size(); ++i) {
        auto index = i;
        std::size_t dest = 0, stride = 1;
        for (std::size_t j = shape.size(); j-- > 0;) {
            dest += (index % full.shape[j] % shape[j]) * stride;
            index /= full.shape[j];
            stride *= shape[j];
        }
        int v = full.coefficients[i];
        out[dest] = static_cast<int>((static_cast<long long>(out[dest]) + v) % Mod);
    }
    return out;
}
} // namespace cp
