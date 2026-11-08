#pragma once

// 旧隣接リスト形式。到達不能は -1。新規コードでは cp::Graph と cp::bfs を使う。
#include "../graph/bfs.hpp"

inline std::vector<int> BFS(const std::vector<std::vector<int>>& adjacency, int start) {
    cp::Graph<int> g(static_cast<int>(adjacency.size()), cp::GraphKind::Directed);
    for (int v = 0; v < g.size(); ++v) for (int to : adjacency[v]) g.add_edge(v, to);
    auto result = cp::bfs(g, start).dist;
    for (int& value : result) if (value == cp::distance_inf<int>()) value = -1;
    return result;
}
