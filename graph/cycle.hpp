#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>
#include "graph.hpp"

namespace cp {
struct Cycle {
    // edges[i] は vertices[i] -> vertices[(i+1)%size()] の辺番号。
    std::vector<int> vertices, edges;
    int size() const { return static_cast<int>(vertices.size()); }
};
// O(V+E)。自己ループ・多重辺に対応。start>=0 ならその頂点から到達する範囲のみ。
template <class T>
std::optional<Cycle> find_cycle(const Graph<T>& g, int start = -1) {
    assert(start == -1 || (0 <= start && start < g.size()));
    std::vector<int> state(g.size(), 0), parent(g.size(), -1), parent_edge(g.size(), -1);
    for (int root = 0; root < g.size(); ++root) {
        int s = start == -1 ? root : start;
        if (state[s] != 0) { if (start != -1) break; else continue; }
        std::vector<std::pair<int, std::size_t>> stack{{s, 0}}; state[s] = 1;
        while (!stack.empty()) {
            int v = stack.back().first;
            auto& next = stack.back().second;
            if (next == g[v].size()) { state[v] = 2; stack.pop_back(); continue; }
            auto e = g[v][next++];
            if (!g.directed() && e.idx == parent_edge[v]) continue;
            if (state[e.to] == 0) {
                parent[e.to] = v; parent_edge[e.to] = e.idx;
                state[e.to] = 1; stack.emplace_back(e.to, 0);
            } else if (state[e.to] == 1) {
                Cycle result;
                for (int cur = v; cur != e.to; cur = parent[cur]) {
                    result.vertices.push_back(cur); result.edges.push_back(parent_edge[cur]);
                }
                result.vertices.push_back(e.to);
                std::reverse(result.vertices.begin(), result.vertices.end());
                std::reverse(result.edges.begin(), result.edges.end());
                result.edges.push_back(e.idx);
                return result;
            }
        }
        if (start != -1) break;
    }
    return std::nullopt;
}
} // namespace cp
