#define PROBLEM "https://judge.yosupo.jp/problem/vertex_add_subtree_sum"
#include <iostream>
#include <vector>
#include "tree/euler_tour.hpp"
#include "range_query/fenwick_tree.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q; std::vector<long long> a(n);
    for (auto& x : a) std::cin >> x;
    cp::Graph<> g(n);
    for (int v = 1, p; v < n; ++v) { std::cin >> p; g.add_edge(p, v); }
    cp::EulerTour<> tour(g); cp::FenwickTree<long long> bit(n);
    for (int v = 0; v < n; ++v) bit.add(tour.tree.tin[v], a[v]);
    while (q--) {
        int type, v; std::cin >> type >> v;
        if (type == 0) { long long x; std::cin >> x; bit.add(tour.tree.tin[v], x); }
        else { auto [l, r] = tour.tree.subtree_interval(v); std::cout << bit.sum(l, r) << '\n'; }
    }
}
