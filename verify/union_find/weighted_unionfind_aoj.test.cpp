#define PROBLEM "https://onlinejudge.u-aizu.ac.jp/problems/DSL_1_B"

#include <iostream>
#include "union_find/potentialized_disjoint_set_union.hpp"

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    int n, q;
    std::cin >> n >> q;
    cp::PotentializedDisjointSetUnion<long long> dsu(n);
    while (q--) {
        int type, u, v;
        std::cin >> type >> u >> v;
        if (type == 0) {
            long long difference;
            std::cin >> difference;
            dsu.merge(u, v, difference);
        } else {
            auto difference = dsu.diff(u, v);
            if (difference) std::cout << *difference << '\n';
            else std::cout << "?\n";
        }
    }
}
