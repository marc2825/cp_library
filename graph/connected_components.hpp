#pragma once
#include <cassert>
#include <vector>
#include "graph.hpp"

namespace cp {
struct ConnectedComponentsResult {
    std::vector<int> component;
    std::vector<std::vector<int>> groups;
};
// 無向グラフ限定。component[v] は groups の添字。O(V+E)。
template <class T>
ConnectedComponentsResult connected_components(const Graph<T>& g) {
    assert(!g.directed());
    ConnectedComponentsResult result;
    result.component.assign(g.size(), -1);
    for (int root = 0; root < g.size(); ++root) {
        if (result.component[root] != -1) continue;
        int id = static_cast<int>(result.groups.size());
        result.groups.push_back({});
        std::vector<int> stack{root}; result.component[root] = id;
        while (!stack.empty()) {
            int v = stack.back(); stack.pop_back();
            result.groups.back().push_back(v);
            for (const auto& e : g[v]) {
                if (result.component[e.to] != -1) continue;
                result.component[e.to] = id; stack.push_back(e.to);
            }
        }
    }
    return result;
}
} // namespace cp
