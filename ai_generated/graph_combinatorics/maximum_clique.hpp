#pragma once
#include <algorithm>
#include <vector>
#include "detail/bit_graph.hpp"

namespace cp {
namespace graph_combinatorics_detail {
inline std::vector<int> maximum_clique_masks(const std::vector<Mask>& adjacency) {
    std::vector<int> current, best;
    auto search = [&](auto&& self, Mask candidates) -> void {
        std::vector<int> order, bound;
        int color = 0;
        for (Mask remaining = candidates; remaining;) {
            ++color;
            for (Mask available = remaining; available;) {
                int v = first_vertex(available);
                remaining &= ~(Mask{1} << v); available &= ~(Mask{1} << v);
                available &= ~adjacency[v];
                order.push_back(v); bound.push_back(color);
            }
        }
        for (int i = static_cast<int>(order.size()) - 1; i >= 0; --i) {
            if (current.size() + bound[i] <= best.size()) return;
            int v = order[i]; current.push_back(v);
            Mask next = candidates & adjacency[v];
            if (next) self(self, next);
            else if (current.size() > best.size()) best = current;
            current.pop_back(); candidates &= ~(Mask{1} << v);
        }
    };
    search(search, all_vertices(static_cast<int>(adjacency.size())));
    std::sort(best.begin(), best.end());
    return best;
}
} // namespace graph_combinatorics_detail

// Exact maximum clique (<=64 vertices), with a greedy-coloring upper bound.
template<class T> std::vector<int> maximum_clique(const Graph<T>& g) {
    return graph_combinatorics_detail::maximum_clique_masks(graph_combinatorics_detail::adjacency_masks(g));
}
} // namespace cp
