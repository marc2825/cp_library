#pragma once
#include <optional>
#include <stdexcept>
#include <vector>
#include "cactus_decomposition.hpp"

namespace cp {
struct UnicyclicDecomposition {
    std::vector<int> cycle, cycle_edge_ids; // cycle repeats its first vertex at the end.
    std::vector<int> root, parent, parent_edge, depth;
    std::vector<std::vector<int>> trees; // cycle order, root included.
};
// Connected undirected unicyclic multigraph; loops and parallel pairs are cycles.
template<class T> std::optional<UnicyclicDecomposition> unicyclic_decomposition(const Graph<T>& g) {
    if (g.directed()) throw std::invalid_argument("unicyclic requires undirected graph");
    int n=g.size(); if (!n || g.edge_count()!=n) return std::nullopt;
    auto cactus=cactus_decomposition(g); if (!cactus.is_cactus) return std::nullopt;
    UnicyclicDecomposition r;
    for (const auto& b:cactus.blocks) if (!b.cycle.empty()) {
        if (!r.cycle.empty()) return std::nullopt;
        r.cycle=b.cycle; r.cycle_edge_ids=b.cycle_edge_ids;
    }
    if (r.cycle.empty()) return std::nullopt;
    std::vector<bool> forbidden(g.edge_count());
    for (int id:r.cycle_edge_ids) forbidden[id]=true;
    r.root.assign(n,-1); r.parent.assign(n,-1); r.parent_edge.assign(n,-1); r.depth.assign(n,0);
    for (std::size_t k=0; k<r.cycle_edge_ids.size(); ++k) {
        int root=r.cycle[k]; r.root[root]=root; std::vector<int> queue{root};
        for (std::size_t i=0; i<queue.size(); ++i) for (auto e:g[queue[i]]) {
            if (forbidden[e.idx] || r.root[e.to]>=0) continue;
            r.root[e.to]=root; r.parent[e.to]=e.from; r.parent_edge[e.to]=e.idx;
            r.depth[e.to]=r.depth[e.from]+1; queue.push_back(e.to);
        }
        r.trees.push_back(std::move(queue));
    }
    for (int root:r.root) if (root<0) return std::nullopt;
    return r;
}
} // namespace cp
