#pragma once
#include "base_polyhedron.hpp"
#include "linear_program.hpp"
#include <optional>
namespace cp {
enum class SubmodularStatus { optimal, iteration_limit, lp_limit };
struct SubmodularOptions {
    std::size_t max_iterations = 256;
    SimplexOptions simplex;
};
struct SubmodularMinimum {
    SubmodularStatus status = SubmodularStatus::iteration_limit;
    BigRational value = 0, lower_bound = 0;
    std::vector<int> selected;
    std::size_t iterations = 0, cuts = 0, oracle_calls = 0;
};
// Exact Lovasz epigraph cutting planes. General submodular oracle, exponential worst case.
// The incumbent and a certified lower bound are preserved on either budget limit.
template <class Oracle>
SubmodularMinimum submodular_minimize(int n, Oracle function, SubmodularOptions options = {}) {
    if (n < 0)
        throw std::invalid_argument("negative submodular ground size");
    using Q = BigRational;
    SubmodularMinimum out;
    auto first = base_polyhedron_greedy(std::vector<Q>(n), function);
    out.oracle_calls = first.oracle_calls;
    out.value = first.offset;
    auto incumbent = [&](const BasePolyhedronResult &g) {
        for (std::size_t i = 0; i < g.chain_values.size(); i++)
            if (g.chain_values[i] < out.value) {
                out.value = g.chain_values[i];
                out.selected.assign(g.order.begin(), g.order.begin() + i);
                std::sort(out.selected.begin(), out.selected.end());
            }
    };
    incumbent(first);
    Q shift = 0;
    for (const auto &x : first.base)
        if (x < Q(0))
            shift += x;
    out.lower_bound = first.offset + shift;
    std::vector<std::vector<Q>> cuts{first.base};
    out.cuts = 1;
    for (;;) {
        if (out.value == out.lower_bound) {
            out.status = SubmodularStatus::optimal;
            return out;
        }
        if (out.iterations == options.max_iterations)
            return out;
        std::vector<std::vector<Q>> a;
        std::vector<Q> b;
        for (int i = 0; i < n; i++) {
            std::vector<Q> row(n + 1);
            row[i] = 1;
            a.push_back(std::move(row));
            b.push_back(1);
        }
        for (auto row : cuts) {
            row.push_back(-1);
            a.push_back(std::move(row));
            b.push_back(shift);
        }
        std::vector<Q> cost(n + 1);
        cost[n] = -1;
        const auto lp = linear_program(a, b, cost, options.simplex);
        ++out.iterations;
        if (lp.status == LPStatus::iteration_limit) {
            out.status = SubmodularStatus::lp_limit;
            return out;
        }
        if (lp.status != LPStatus::optimal)
            throw std::logic_error("invalid submodular epigraph LP");
        const Q theta = shift + lp.primal[n];
        out.lower_bound = first.offset + theta;
        auto x = lp.primal;
        x.resize(n);
        auto greedy = base_polyhedron_greedy(x, function);
        out.oracle_calls += greedy.oracle_calls;
        incumbent(greedy);
        if (out.lower_bound > out.value || greedy.value < theta)
            throw std::invalid_argument("oracle is inconsistent or not submodular");
        if (out.value == out.lower_bound) {
            out.status = SubmodularStatus::optimal;
            return out;
        }
        if (greedy.value == theta)
            throw std::invalid_argument("oracle is inconsistent or not submodular");
        cuts.push_back(std::move(greedy.base));
        out.cuts = cuts.size();
    }
}
struct SubmodularGreedyMaximum {
    BigRational value = 0;
    std::vector<int> selected;
    std::size_t oracle_calls = 0;
};
// Monotone submodular, F(empty)=0. At most k elements, 1-(1-1/k)^k guarantee for k>0.
template <class Oracle> SubmodularGreedyMaximum submodular_greedy_cardinality(int n, int k, Oracle function) {
    if (n < 0 || k < 0)
        throw std::invalid_argument("negative cardinality");
    k = std::min(k, n);
    SubmodularGreedyMaximum out;
    out.value = function(std::vector<int>{});
    ++out.oracle_calls;
    if (out.value != BigRational(0))
        throw std::invalid_argument("greedy maximum needs F(empty)=0");
    std::vector<bool> used(n);
    for (int step = 0; step < k; step++) {
        int best = -1;
        BigRational value = out.value;
        for (int e = 0; e < n; e++)
            if (!used[e]) {
                auto next = out.selected;
                next.push_back(e);
                const BigRational score = function(next);
                ++out.oracle_calls;
                if (best < 0 || score > value) {
                    best = e;
                    value = score;
                }
            }
        if (value < out.value)
            throw std::invalid_argument("nonmonotone submodular oracle");
        out.value = value;
        out.selected.push_back(best);
        used[best] = true;
    }
    std::sort(out.selected.begin(), out.selected.end());
    return out;
}
} // namespace cp
