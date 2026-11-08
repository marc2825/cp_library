#pragma once
#include <optional>
#include <stdexcept>
#include <vector>
#include "../graph/graph.hpp"
#include "../combinatorial_optimization/general_matching.hpp"
#include "../graph_flow/bipartite_vertex_cover.hpp"

namespace cp {
namespace detail {
template<class T> std::optional<std::vector<int>> edge_cover_from_mates(const Graph<T>& g, const std::vector<int>& mate) {
    std::vector<int> selected(g.size(),-1), answer;
    for (const auto& e:g.edges()) if (mate[e.from]==e.to && selected[e.from]<0) {
        selected[e.from]=selected[e.to]=e.idx; answer.push_back(e.idx);
    }
    for (int v=0; v<g.size(); ++v) if (selected[v]<0) {
        if (g[v].empty()) return std::nullopt;
        auto e=g[v][0]; selected[v]=selected[e.to]=e.idx; answer.push_back(e.idx);
    }
    return answer;
}
}
// Loopless undirected multigraph; nullopt iff an isolated vertex exists.
template<class T> std::optional<std::vector<int>> minimum_edge_cover(const Graph<T>& g) {
    if (g.directed()) throw std::invalid_argument("edge cover requires undirected graph");
    GeneralMatching matcher(g.size());
    for (const auto& e:g.edges()) {
        if (e.from==e.to) throw std::invalid_argument("edge cover requires loopless graph");
        matcher.add_edge(e.from,e.to);
    }
    return detail::edge_cover_from_mates(g,matcher.maximum_matching().mate);
}
// IDs are indices into edges; left and right have separate local indices.
inline std::optional<std::vector<int>> bipartite_minimum_edge_cover(
        int left, int right, const std::vector<std::pair<int,int>>& edges) {
    auto matching=bipartite_vertex_cover(left,right,edges).matching;
    Graph<int> g(left+right);
    for (auto [u,v]:edges) g.add_edge(u,left+v);
    std::vector<int> mate(left+right,-1);
    for (auto [u,v]:matching.pairs) {mate[u]=left+v; mate[left+v]=u;}
    return detail::edge_cover_from_mates(g,mate);
}
} // namespace cp
