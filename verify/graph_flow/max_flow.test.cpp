#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_6_A"
#include <iostream>
#include "graph_flow/dinic.hpp"
int main() {
    int n, m; std::cin >> n >> m;
    cp::Dinic<> graph(n);
    for (int i = 0; i < m; ++i) {
        int u, v; long long capacity; std::cin >> u >> v >> capacity;
        graph.add_edge(u, v, capacity);
    }
    std::cout << graph.max_flow(0, n - 1) << '\n';
}
