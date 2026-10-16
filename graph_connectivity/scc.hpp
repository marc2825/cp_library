#pragma once
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>
#include "graph.hpp"

// Iterative Kosaraju. Component IDs follow a topological order of the condensation.
// Members of each group are in vertex order. Edge weights are ignored.
// Docs: docs/graph_connectivity/scc.md
namespace cp {

struct StronglyConnectedComponentsResult {
    std::vector<int> component;
    std::vector<std::vector<int>> groups;
    int size() const { return static_cast<int>(groups.size()); }
    bool same(int u, int v) const {
        assert(0 <= u && u < static_cast<int>(component.size()));
        assert(0 <= v && v < static_cast<int>(component.size()));
        return component[u] == component[v];
    }
};

template <class T>
StronglyConnectedComponentsResult strongly_connected_components(const Graph<T>& g) {
    assert(g.directed());
    int n = g.size();
    std::vector<std::vector<int>> reverse(n);
    for (const auto& e : g.edges()) reverse[e.to].push_back(e.from);
    std::vector<bool> visited(n, false);
    std::vector<int> finish;
    finish.reserve(n);
    for (int root = 0; root < n; ++root) {
        if (visited[root]) continue;
        std::vector<std::pair<int, std::size_t>> stack{{root, 0}};
        visited[root] = true;
        while (!stack.empty()) {
            int v = stack.back().first;
            auto& next = stack.back().second;
            if (next == g[v].size()) { finish.push_back(v); stack.pop_back(); continue; }
            int to = g[v][next++].to;
            if (visited[to]) continue;
            visited[to] = true;
            stack.emplace_back(to, 0);
        }
    }
    StronglyConnectedComponentsResult result;
    result.component.assign(n, -1);
    int count = 0;
    for (auto it = finish.rbegin(); it != finish.rend(); ++it) {
        if (result.component[*it] != -1) continue;
        std::vector<int> stack{*it};
        result.component[*it] = count;
        while (!stack.empty()) {
            int v = stack.back(); stack.pop_back();
            for (int to : reverse[v]) {
                if (result.component[to] != -1) continue;
                result.component[to] = count;
                stack.push_back(to);
            }
        }
        ++count;
    }
    result.groups.resize(count);
    for (int v = 0; v < n; ++v) result.groups[result.component[v]].push_back(v);
    return result;
}

template <class T>
StronglyConnectedComponentsResult scc(const Graph<T>& g) {
    return strongly_connected_components(g);
}

} // namespace cp
