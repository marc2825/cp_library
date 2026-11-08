#pragma once
#include "detail/arithmetic.hpp"
#include <climits>
#include <vector>
namespace cp {
struct WeightedMatroidSet {
    OptimizationWide weight = 0;
    std::vector<int> selected;
};
struct WeightedMatroidIntersectionResult : WeightedMatroidSet {
    std::vector<WeightedMatroidSet> by_size; // maximum weight for every feasible cardinality, including zero
    std::size_t oracle_calls = 0;
};
// Minimum-cost augmentations with Bellman-Ford, breaking cost ties by the number of edges.
// Oracle contract matches matroid_intersection. Negative weights are allowed.
template <class First, class Second>
WeightedMatroidIntersectionResult weighted_matroid_intersection(const std::vector<long long> &weights,
                                                                First first, Second second) {
    if (weights.size() > static_cast<std::size_t>(INT_MAX - 2))
        throw std::length_error("too many matroid elements");
    const int n = static_cast<int>(weights.size()), source = n, sink = n + 1;
    WeightedMatroidIntersectionResult out;
    out.oracle_calls = 2;
    if (!first(std::vector<int>{}) || !second(std::vector<int>{}))
        throw std::invalid_argument("matroids must contain empty set");
    out.by_size.push_back({});
    std::vector<bool> selected(n);
    for (;;) {
        std::vector<int> inside, outside;
        for (int e = 0; e < n; e++)
            (selected[e] ? inside : outside).push_back(e);
        struct Arc {
            int from, to;
            OptimizationWide cost;
        };
        std::vector<Arc> arcs;
        auto test = [&](auto &oracle, int remove, int add) {
            auto set = inside;
            if (remove >= 0)
                set.erase(std::find(set.begin(), set.end(), remove));
            set.push_back(add);
            ++out.oracle_calls;
            return oracle(set);
        };
        for (int e : outside) {
            if (test(first, -1, e))
                arcs.push_back({source, e, -OptimizationWide(weights[e])});
            if (test(second, -1, e))
                arcs.push_back({e, sink, 0});
            for (int x : inside) {
                if (test(first, x, e))
                    arcs.push_back({x, e, -OptimizationWide(weights[e])});
                if (test(second, x, e))
                    arcs.push_back({e, x, OptimizationWide(weights[x])});
            }
        }
        std::vector<OptimizationWide> distance(n + 2);
        std::vector<int> length(n + 2), parent(n + 2, -1);
        std::vector<bool> reached(n + 2);
        reached[source] = true;
        for (int pass = 0; pass < n + 2; pass++) {
            bool changed = false;
            for (const auto &edge : arcs)
                if (reached[edge.from]) {
                    const auto next = optimization_detail::add(distance[edge.from], edge.cost);
                    const int hops = length[edge.from] + 1;
                    if (!reached[edge.to] || next < distance[edge.to] ||
                        (next == distance[edge.to] && hops < length[edge.to])) {
                        reached[edge.to] = true;
                        distance[edge.to] = next;
                        length[edge.to] = hops;
                        parent[edge.to] = edge.from;
                        changed = true;
                    }
                }
            if (!changed)
                break;
            if (pass == n + 1)
                throw std::logic_error("negative matroid exchange cycle: invalid independence oracle");
        }
        if (!reached[sink])
            break;
        int steps = 0;
        for (int e = parent[sink]; e != source; e = parent[e]) {
            if (e < 0 || e >= n || ++steps > n)
                throw std::logic_error("invalid matroid augmenting path");
            selected[e] = !selected[e];
        }
        WeightedMatroidSet current;
        for (int e = 0; e < n; e++)
            if (selected[e]) {
                current.selected.push_back(e);
                current.weight = optimization_detail::add(current.weight, weights[e]);
            }
        out.oracle_calls += 2;
        if (current.selected.size() != out.by_size.size() || !first(current.selected) ||
            !second(current.selected))
            throw std::logic_error("invalid matroid augmentation");
        if (current.weight >= out.weight) {
            out.weight = current.weight;
            out.selected = current.selected;
        }
        out.by_size.push_back(std::move(current));
    }
    return out;
}
} // namespace cp
