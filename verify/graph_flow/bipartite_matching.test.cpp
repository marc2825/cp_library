#define PROBLEM "https://judge.yosupo.jp/problem/bipartitematching"
#include <iostream>
#include "graph_flow/bipartite_matching.hpp"
int main() {
    int left, right, m; std::cin >> left >> right >> m;
    cp::BipartiteMatching graph(left, right);
    for (int i = 0; i < m; ++i) { int u, v; std::cin >> u >> v; graph.add_edge(u, v); }
    const auto result = graph.maximum_matching();
    std::cout << result.cardinality << '\n';
    for (auto [u, v] : result.pairs) std::cout << u << ' ' << v << '\n';
}
