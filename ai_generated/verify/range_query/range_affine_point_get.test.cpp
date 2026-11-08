#define PROBLEM "https://judge.yosupo.jp/problem/range_affine_point_get"
#include "range_query/dual_segtree.hpp"
#include "test_values.hpp"
#include <iostream>
#include <vector>
using namespace range_test;
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q;
    std::vector<long long> a(n); for (auto& x : a) std::cin >> x;
    cp::DualSegtree<long long, Affine, range_test::apply, compose, identity> tree(a);
    while (q--) {
        int t, l; std::cin >> t >> l;
        if (t == 0) { int r; Affine f; std::cin >> r >> f.a >> f.b; tree.apply(l, r, f); }
        else std::cout << tree.get(l) << '\n';
    }
}
