#pragma once
#include <vector>
#include "detail/product_tree.hpp"

namespace cp {
// xsの順でa(xs[i])。重複点も可。空の点列は空。
template <int Mod = 998244353, int Root = 3> std::vector<int> multipoint_evaluation(const std::vector<int>& a, const std::vector<int>& xs) {
    if (xs.empty()) return {};
    return fps_detail::ProductTree<Mod, Root>(xs).evaluate(a);
}
} // namespace cp
