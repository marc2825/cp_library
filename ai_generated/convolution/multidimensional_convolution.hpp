#pragma once
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
#include "convolution.hpp"

namespace cp {
struct MultidimensionalConvolutionResult {
    std::vector<std::size_t> shape;
    std::vector<int> coefficients;
};
// 行優先（最後の次元が最速）。各次元は正、0次元は長さ1のスカラー。
// Kronecker置換による通常の多次元積。巡回積ではない。
template <int Mod = 998244353, int Root = 3>
MultidimensionalConvolutionResult multidimensional_convolution(
    const std::vector<int>& a, const std::vector<std::size_t>& a_shape,
    const std::vector<int>& b, const std::vector<std::size_t>& b_shape) {
    if (a_shape.size() != b_shape.size()) throw std::invalid_argument("tensor dimensions differ");
    auto volume = [](const std::vector<std::size_t>& shape) {
        std::size_t result = 1;
        for (auto length : shape) {
            if (!length) throw std::invalid_argument("tensor axes must be positive");
            if (result > std::vector<int>().max_size() / length) throw std::length_error("tensor volume overflow");
            result *= length;
        }
        return result;
    };
    if (volume(a_shape) != a.size() || volume(b_shape) != b.size()) throw std::invalid_argument("tensor storage size mismatch");
    std::vector<std::size_t> shape(a_shape.size()), strides(shape.size());
    for (std::size_t i = 0; i < shape.size(); ++i) {
        if (a_shape[i] > std::numeric_limits<std::size_t>::max() - (b_shape[i] - 1)) throw std::length_error("tensor axis overflow");
        shape[i] = a_shape[i] + b_shape[i] - 1;
    }
    const auto count = volume(shape);
    if (count > ntt_max_size<Mod, Root>()) throw std::length_error("tensor product exceeds NTT capacity");
    std::size_t stride = 1;
    for (std::size_t i = shape.size(); i-- > 0;) { strides[i] = stride; stride *= shape[i]; }
    auto encode = [&](const std::vector<int>& f, const std::vector<std::size_t>& dimensions) {
        std::size_t maximum = 0;
        for (std::size_t i = 0; i < dimensions.size(); ++i) maximum += (dimensions[i] - 1) * strides[i];
        std::vector<int> encoded(maximum + 1);
        for (std::size_t i = 0; i < f.size(); ++i) {
            auto index = i; std::size_t target = 0;
            for (std::size_t d = dimensions.size(); d-- > 0;) { target += (index % dimensions[d]) * strides[d]; index /= dimensions[d]; }
            encoded[target] = f[i];
        }
        return encoded;
    };
    auto result = convolution<Mod, Root>(encode(a, a_shape), encode(b, b_shape));
    result.resize(count);
    return {shape, result};
}
} // namespace cp
