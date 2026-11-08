#define PROBLEM "https://judge.yosupo.jp/problem/lca"
#include <iostream>
#include "tree/doubling_lca.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q; cp::Graph<> g(n);
    for (int v = 1, p; v < n; ++v) { std::cin >> p; g.add_edge(p, v); }
    cp::DoublingLCA<> lca(g);
    while (q--) { int u, v; std::cin >> u >> v; std::cout << lca.lca(u, v) << '\n'; }
}
