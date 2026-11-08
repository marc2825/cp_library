#pragma once
#include "../basic_utility/big_rational.hpp"
#include <algorithm>
#include <climits>
#include <numeric>
#include <vector>
namespace cp {
struct BasePolyhedronResult {
    std::vector<BigRational> base, chain_values;
    std::vector<int> order;
    BigRational offset = 0, value = 0;
    std::size_t oracle_calls = 0;
};
// Linear optimization over B(F-F(empty)). F is a deterministic submodular set-function.
// Negative marginals are allowed; the oracle accepts distinct element IDs in arbitrary order.
template <class Oracle>
BasePolyhedronResult base_polyhedron_greedy(const std::vector<BigRational> &weights, Oracle function,
                                            bool maximize = true) {
    if (weights.size() > static_cast<std::size_t>(INT_MAX))
        throw std::length_error("base ground set too large");
    BasePolyhedronResult out;
    out.base.resize(weights.size());
    out.order.resize(weights.size());
    std::iota(out.order.begin(), out.order.end(), 0);
    std::sort(out.order.begin(), out.order.end(), [&](int a, int b) {
        return weights[a] != weights[b] ? (maximize ? weights[a] > weights[b] : weights[a] < weights[b])
                                        : a < b;
    });
    std::vector<int> prefix;
    out.offset = function(prefix);
    out.chain_values.push_back(out.offset);
    ++out.oracle_calls;
    for (int e : out.order) {
        prefix.push_back(e);
        const BigRational next = function(prefix);
        ++out.oracle_calls;
        out.base[e] = next - out.chain_values.back();
        out.chain_values.push_back(next);
        out.value += weights[e] * out.base[e];
    }
    return out;
}
template <class Oracle> BigRational lovasz_extension(const std::vector<BigRational> &x, Oracle function) {
    auto result = base_polyhedron_greedy(x, std::move(function));
    return result.offset + result.value;
}
} // namespace cp
