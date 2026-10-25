#pragma once
#include <stdexcept>
#include <utility>
#include <vector>
#include "bipartite_matching.hpp"

namespace cp {
struct BipartiteVertexCoverResult {
    BipartiteMatchingResult matching;
    std::vector<int> left_cover, right_cover, left_independent, right_independent;
    std::vector<bool> reachable_left, reachable_right;
};
// König: alternating reachability from unmatched left vertices. O((V+E)sqrt(V)).
inline BipartiteVertexCoverResult bipartite_vertex_cover(
        int left, int right, const std::vector<std::pair<int,int>>& edges) {
    if (left < 0 || right < 0) throw std::invalid_argument("negative bipartite size");
    BipartiteMatching matcher(left, right);
    std::vector<std::vector<int>> adj(left);
    for (auto [u,v] : edges) {
        if (u < 0 || u >= left || v < 0 || v >= right) throw std::out_of_range("bipartite vertex");
        matcher.add_edge(u,v); adj[u].push_back(v);
    }
    BipartiteVertexCoverResult r;
    r.matching = matcher.maximum_matching();
    r.reachable_left.assign(left,false); r.reachable_right.assign(right,false);
    std::vector<int> queue;
    for (int u=0; u<left; ++u) if (r.matching.left_match[u]<0) {
        r.reachable_left[u]=true; queue.push_back(u);
    }
    for (std::size_t i=0; i<queue.size(); ++i) for (int v:adj[queue[i]]) {
        if (r.reachable_right[v]) continue;
        r.reachable_right[v]=true;
        int u=r.matching.right_match[v];
        if (u>=0 && !r.reachable_left[u]) { r.reachable_left[u]=true; queue.push_back(u); }
    }
    for (int u=0; u<left; ++u) (r.reachable_left[u]?r.left_independent:r.left_cover).push_back(u);
    for (int v=0; v<right; ++v) (r.reachable_right[v]?r.right_cover:r.right_independent).push_back(v);
    return r;
}
} // namespace cp
