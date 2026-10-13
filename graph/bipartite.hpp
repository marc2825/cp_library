#pragma once
#include <cassert>
#include <optional>
#include <queue>
#include <vector>
#include "graph.hpp"

namespace cp {
// 無向グラフ限定。二部グラフなら 0/1 の色、そうでなければ nullopt。O(V+E)。
template <class T>
std::optional<std::vector<int>> bipartite_coloring(const Graph<T>& g) {
    assert(!g.directed());
    std::vector<int> color(g.size(), -1);
    std::queue<int> queue;
    for (int root = 0; root < g.size(); ++root) {
        if (color[root] != -1) continue;
        color[root] = 0; queue.push(root);
        while (!queue.empty()) {
            int v = queue.front(); queue.pop();
            for (const auto& e : g[v]) {
                if (color[e.to] == color[v]) return std::nullopt;
                if (color[e.to] != -1) continue;
                color[e.to] = color[v] ^ 1; queue.push(e.to);
            }
        }
    }
    return color;
}
} // namespace cp
