#define PROBLEM "https://judge.yosupo.jp/problem/unionfind_with_potential"

#include <iostream>
#include "union_find/potentialized_disjoint_set_union.hpp"
#include "test_values.hpp"

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    int n, q;
    std::cin >> n >> q;
    cp::PotentializedDisjointSetUnion<union_find_test::ModValue> dsu(n);
    while (q--) {
        int type, u, v;
        std::cin >> type >> u >> v;
        // この問題は a[u]-a[v] の向き。ライブラリの引数を逆にして合わせる。
        if (type == 0) {
            int difference;
            std::cin >> difference;
            auto result = dsu.merge(v, u, union_find_test::ModValue(difference));
            std::cout << (result != cp::PotentialMergeResult::Contradiction) << '\n';
        } else {
            auto difference = dsu.diff(v, u);
            std::cout << (difference ? difference->value : -1) << '\n';
        }
    }
}
