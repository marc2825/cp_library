#define PROBLEM "https://judge.yosupo.jp/problem/point_add_range_sum"
#include "range_query/fenwick_tree.hpp"
#include <iostream>
#include <vector>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q;
    std::vector<long long> a(n); for (auto& x : a) std::cin >> x;
    cp::FenwickTree<long long> tree(a);
    while (q--) {
        int t, l; long long r; std::cin >> t >> l >> r;
        if (t == 0) tree.add(l, r);
        else std::cout << tree.sum(l, int(r)) << '\n';
    }
}
