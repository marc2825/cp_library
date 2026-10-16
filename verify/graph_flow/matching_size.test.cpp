#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_7_A"
#include <iostream>
#include "graph_flow/bipartite_matching.hpp"
int main() {
    int left, right, m; std::cin >> left >> right >> m;
    cp::BipartiteMatching graph(left, right);
    for (int i = 0; i < m; ++i) { int u, v; std::cin >> u >> v; graph.add_edge(u, v); }
    std::cout << graph.maximum_matching().cardinality << '\n';
}
