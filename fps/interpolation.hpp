#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "detail/product_tree.hpp"
#include "polynomial.hpp"

namespace cp {
// 相異なるxs[i]においてf(xs[i])=ys[i]。長さxs.size()（末尾0も保持）。
template <int Mod = 998244353, int Root = 3> std::vector<int> polynomial_interpolation(const std::vector<int>& xs, const std::vector<int>& ys) {
    using F = fps_detail::Field<Mod>;
    if (xs.size() != ys.size()) throw std::invalid_argument("interpolation input lengths differ");
    if (xs.empty()) return {};
    const fps_detail::ProductTree<Mod, Root> tree(xs);
    auto denominators = tree.evaluate(polynomial_derivative<Mod>(tree.nodes[1]));
    std::vector<std::vector<int>> work(2 * tree.base);
    // M'(xs[i])=0なら法で同じ点がある。全て検査してからバッチ逆元を計算。
    std::vector<int> prefix(xs.size() + 1, 1);
    for (std::size_t i = 0; i < xs.size(); ++i) {
        if (!denominators[i]) throw std::domain_error("interpolation points must be distinct modulo modulus");
        prefix[i + 1] = F::mul(prefix[i], denominators[i]);
    }
    int inverse = F::inv(prefix.back());
    for (std::size_t i = tree.base; i-- > 0;) {
        if (i < xs.size()) {
            work[tree.base + i] = {F::mul(F::norm(ys[i]), F::mul(inverse, prefix[i]))};
            inverse = F::mul(inverse, denominators[i]);
        } else work[tree.base + i] = {0};
    }
    for (std::size_t i = tree.base; --i > 0;) {
        work[i] = polynomial_add<Mod>(fps_detail::multiply<Mod, Root>(work[2 * i], tree.nodes[2 * i + 1]),
                                     fps_detail::multiply<Mod, Root>(work[2 * i + 1], tree.nodes[2 * i]));
    }
    work[1].resize(xs.size());
    return work[1];
}
} // namespace cp
