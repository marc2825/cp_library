#include <iostream>
#include "union_find/disjoint_set_union.hpp"
int main() {
    int n, m;
    if (!(std::cin >> n >> m)) return 1;
    cp::DisjointSetUnion dsu(n);
    for (int i = 0, u, v; i < m; ++i) {
        std::cin >> u >> v;
        dsu.merge(u, v);
    }
    int components = 0;
    for (int i = 0; i < n; ++i) if (dsu.leader(i) == i) ++components;
    std::cout << components << '\n';
}
