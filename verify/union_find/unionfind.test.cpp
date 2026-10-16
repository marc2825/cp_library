#define PROBLEM "https://judge.yosupo.jp/problem/unionfind"

#include <iostream>
#include "union_find/disjoint_set_union.hpp"

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    int n, q;
    std::cin >> n >> q;
    cp::DisjointSetUnion dsu(n);
    while (q--) {
        int type, u, v;
        std::cin >> type >> u >> v;
        if (type == 0) dsu.merge(u, v);
        else std::cout << dsu.same(u, v) << '\n';
    }
}
