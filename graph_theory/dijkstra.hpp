#pragma once

// 旧単独 Dijkstra の互換入口。Graph<T> のコスト型は従来の修正どおり保持する。
// Graph<T> g(n); g[u].emplace_back(v, cost); auto dist = dijkstra(g, s);
// 到達不能は従来どおり -1。新規コードは graph/dijkstra.hpp と cp::dijkstra を使う。
#include "graph_template.hpp"

template <class T>
std::vector<T> dijkstra(const Graph<T>& g, int start = 0) {
    auto result = g.dijkstra(start).dist;
    for (auto& value : result) if (value == cp::distance_inf<T>()) value = T(-1);
    return result;
}
