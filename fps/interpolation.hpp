#pragma once
#include <vector>
#include "subproduct_tree.hpp"

namespace cp {
// 点数と同じ長さを返す。法で相異なる点が必要。
template <int Mod = 998244353, int Root = 3> std::vector<int> polynomial_interpolation(const std::vector<int>& xs, const std::vector<int>& ys) {
    if (xs.size() != ys.size()) throw std::invalid_argument("interpolation input lengths differ");
    return SubproductTree<Mod, Root>(xs).interpolate(ys);
}
} // namespace cp
