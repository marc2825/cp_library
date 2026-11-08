#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "detail/product_tree.hpp"

namespace cp {
// 構築済みの積木を繰返し評価・補間に使う。点は評価時には重複可。
template <int Mod = 998244353, int Root = 3> class SubproductTree {
    using F = fps_detail::Field<Mod>;
    fps_detail::ProductTree<Mod, Root> tree_;
public:
    explicit SubproductTree(const std::vector<int>& points) : tree_(points) {}
    std::size_t size() const { return tree_.count; }
    const std::vector<int>& product() const { return tree_.nodes[1]; }
    std::vector<int> evaluate(const std::vector<int>& f) const { return tree_.evaluate(f); }
    std::vector<int> interpolate(const std::vector<int>& values) const {
        if (values.size() != size()) throw std::invalid_argument("interpolation value count mismatch");
        if (!size()) return {};
        const auto denominators = evaluate(polynomial_derivative<Mod>(product()));
        std::vector<int> prefix(size() + 1, 1);
        for (std::size_t i = 0; i < size(); ++i) {
            if (!denominators[i]) throw std::domain_error("interpolation points must be distinct");
            prefix[i + 1] = F::mul(prefix[i], denominators[i]);
        }
        int inverse = F::inv(prefix.back());
        std::vector<std::vector<int>> work(2 * tree_.base, std::vector<int>{0});
        for (std::size_t i = size(); i-- > 0;) {
            work[tree_.base + i] = {F::mul(F::norm(values[i]), F::mul(inverse, prefix[i]))};
            inverse = F::mul(inverse, denominators[i]);
        }
        for (std::size_t i = tree_.base; --i > 0;)
            work[i] = polynomial_add<Mod>(fps_detail::multiply<Mod, Root>(work[2 * i], tree_.nodes[2 * i + 1]),
                                         fps_detail::multiply<Mod, Root>(work[2 * i + 1], tree_.nodes[2 * i]));
        work[1].resize(size()); return work[1];
    }
};
} // namespace cp
