#pragma once
#include "../game_theory/nimber.hpp"
#include "karatsuba.hpp"
#include <cstdint>
#include <vector>
namespace cp {
// GF(2^64) in the existing Conway nimber basis, Karatsuba O(n^log2(3)) field operations.
inline std::vector<std::uint64_t> convolution_nimber(const std::vector<std::uint64_t> &a,
                                                     const std::vector<std::uint64_t> &b) {
    std::vector<Nimber> x, y;
    for (auto v : a)
        x.emplace_back(v);
    for (auto v : b)
        y.emplace_back(v);
    auto z = convolution_karatsuba(x, y);
    std::vector<std::uint64_t> out;
    out.reserve(z.size());
    for (auto v : z)
        out.push_back(v.value());
    return out;
}
} // namespace cp
