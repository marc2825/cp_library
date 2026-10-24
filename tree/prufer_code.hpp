#pragma once
// 最小ラベルの葉を除くPrüfer符号。説明: docs/tree/prufer_code.md
#include <functional>
#include <queue>
#include <stdexcept>
#include <vector>
#include "rooted_tree.hpp"

namespace cp {
// 重みを無視。空は拒否。n=1,2 はどちらも空の符号を返す。
template<class T> std::vector<int> prufer_encode(const Graph<T>& g) {
    if (g.size() == 0) throw std::invalid_argument("Pruefer tree must be nonempty");
    Graph<int> topology(g.size(), g.kind());
    for (auto e : g.edges()) topology.add_edge(e.from, e.to);
    RootedTree<int> tree(topology);
    if (tree.roots.size() != 1) throw std::invalid_argument("Pruefer tree must be connected");
    std::vector<int> degree(g.size()), code;
    std::priority_queue<int, std::vector<int>, std::greater<int>> leaves;
    for (int v = 0; v < g.size(); ++v) {
        degree[v] = static_cast<int>(g[v].size());
        if (degree[v] == 1) leaves.push(v);
    }
    for (int i = 0; i < g.size() - 2; ++i) {
        int v = leaves.top(); leaves.pop(); degree[v] = 0;
        for (auto e : g[v]) if (degree[e.to]) {
            code.push_back(e.to);
            if (--degree[e.to] == 1) leaves.push(e.to);
            break;
        }
    }
    return code;
}

// nを明示してn=1とn=2を区別。単位重みの木。O(n log n)。
inline Graph<int> prufer_decode(int n, const std::vector<int>& code) {
    if (n <= 0 || code.size() != static_cast<std::size_t>(n >= 2 ? n - 2 : 0))
        throw std::invalid_argument("invalid Pruefer length");
    std::vector<int> degree(n, 1);
    for (int v : code) {
        if (v < 0 || v >= n) throw std::invalid_argument("invalid Pruefer label");
        ++degree[v];
    }
    Graph<int> g(n);
    if (n == 1) return g;
    std::priority_queue<int, std::vector<int>, std::greater<int>> leaves;
    for (int v = 0; v < n; ++v) if (degree[v] == 1) leaves.push(v);
    for (int v : code) {
        int u = leaves.top(); leaves.pop(); g.add_edge(u, v);
        --degree[u];
        if (--degree[v] == 1) leaves.push(v);
    }
    int u = leaves.top(); leaves.pop(); g.add_edge(u, leaves.top());
    return g;
}
} // namespace cp
