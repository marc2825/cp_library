#pragma once
#include "quotient_prefix.hpp"

namespace cp {
// Dirichlet hyperbola identity at every floor quotient. O(N^(3/4)) ring operations.
template<class T> QuotientPrefix<T> dirichlet_prefix_product(const QuotientPrefix<T>& f, const QuotientPrefix<T>& g) {
    if (f.limit() != g.limit()) throw std::invalid_argument("Dirichlet table limits differ");
    using namespace quotient_detail;
    return QuotientPrefix<T>(f.limit(), [&](std::uint64_t x) {
        const auto root = integer_kth_root(x, 2);
        T result = T(0);
        for (std::uint64_t i = 1; i <= root; ++i) {
            result = add(result, mul(sub(f.sum(i), f.sum(i - 1)), g.sum(x / i)));
            result = add(result, mul(sub(g.sum(i), g.sum(i - 1)), f.sum(x / i)));
        }
        return sub(result, mul(f.sum(root), g.sum(root)));
    }, f.root());
}
} // namespace cp
