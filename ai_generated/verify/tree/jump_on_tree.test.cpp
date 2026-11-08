#define PROBLEM "https://judge.yosupo.jp/problem/jump_on_tree"
#include <iostream>
#include "tree/doubling_lca.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q; cp::Graph<> g(n);
    for (int i = 1, u, v; i < n; ++i) { std::cin >> u >> v; g.add_edge(u, v); }
    cp::DoublingLCA<> lca(g);
    while (q--) { int s, t, k; std::cin >> s >> t >> k; std::cout << lca.jump(s, t, k) << '\n'; }
}
