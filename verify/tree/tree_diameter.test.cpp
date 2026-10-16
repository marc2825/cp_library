#define PROBLEM "https://judge.yosupo.jp/problem/tree_diameter"
#include <iostream>
#include "tree/diameter.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n; cp::Graph<> g(n);
    for (int i = 1, u, v; i < n; ++i) { long long w; std::cin >> u >> v >> w; g.add_edge(u, v, w); }
    auto result = cp::tree_diameter(g);
    std::cout << result.length << ' ' << result.vertices.size() << '\n';
    for (int v : result.vertices) std::cout << v << ' ';
    std::cout << '\n';
}
