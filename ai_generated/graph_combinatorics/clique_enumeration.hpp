#pragma once
#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>
#include "detail/bit_graph.hpp"

namespace cp {
// All cliques, each once. Copy the callback's temporary vertex vector if storing it.
template<class T, class Emit>
bool enumerate_cliques(const Graph<T>& g, Emit emit, bool include_empty = false) {
    using namespace graph_combinatorics_detail;
    assert_simple_undirected(g);
    std::vector<std::vector<int>> forward(g.size());
    for (const auto& e : g.edges()) forward[std::min(e.from, e.to)].push_back(std::max(e.from, e.to));
    for (auto& row : forward) std::sort(row.begin(), row.end());
    std::vector<int> clique;
    if (include_empty && !emit(std::as_const(clique))) return false;
    auto search = [&](auto&& self, const std::vector<int>& candidates) -> bool {
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            int v = candidates[i]; clique.push_back(v);
            if (!emit(std::as_const(clique))) return false;
            if (!forward[v].empty() && i + 1 < candidates.size()) {
                std::vector<int> next;
                std::set_intersection(candidates.begin() + i + 1, candidates.end(),
                                      forward[v].begin(), forward[v].end(), std::back_inserter(next));
                if (!self(self, next)) return false;
            }
            clique.pop_back();
        }
        return true;
    };
    for (int v = 0; v < g.size(); ++v) {
        clique.push_back(v);
        if (!emit(std::as_const(clique)) || !search(search, forward[v])) return false;
        clique.pop_back();
    }
    return true;
}

// Inclusion-maximal cliques via pivoted Bron-Kerbosch; not just the largest ones.
template<class T, class Emit>
bool enumerate_maximal_cliques(const Graph<T>& g, Emit emit, bool include_empty = false) {
    using namespace graph_combinatorics_detail;
    const auto adjacency = adjacency_masks(g);
    std::vector<int> clique;
    auto search = [&](auto&& self, Mask candidates, Mask excluded) -> bool {
        if (!(candidates | excluded)) {
            if (clique.empty() && !include_empty) return true;
            auto sorted = clique; std::sort(sorted.begin(), sorted.end());
            return emit(std::as_const(sorted));
        }
        int pivot = -1, score = -1;
        for (Mask pool = candidates | excluded; pool; pool &= pool - 1) {
            int v = first_vertex(pool), count = bit_count(candidates & adjacency[v]);
            if (count > score) { score = count; pivot = v; }
        }
        Mask branches = candidates & ~adjacency[pivot];
        while (branches) {
            int v = first_vertex(branches); branches &= branches - 1;
            clique.push_back(v);
            if (!self(self, candidates & adjacency[v], excluded & adjacency[v])) return false;
            clique.pop_back();
            candidates &= ~(Mask{1} << v); excluded |= Mask{1} << v;
        }
        return true;
    };
    return search(search, all_vertices(g.size()), Mask{});
}
} // namespace cp
