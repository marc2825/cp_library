#define PROBLEM "https://judge.yosupo.jp/problem/range_affine_range_sum"
#include "range_query/lazy_segtree.hpp"
#include "test_values.hpp"
#include <iostream>
#include <vector>
using namespace range_test;
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q;
    std::vector<Sum> a(n); for (auto& x : a) { std::cin >> x.value; x.length = 1; }
    cp::LazySegtree<Sum, combine, range_test::empty, Affine, map_sum, compose, identity> tree(a);
    while (q--) {
        int t, l, r; std::cin >> t >> l >> r;
        if (t == 0) { Affine f; std::cin >> f.a >> f.b; tree.apply(l, r, f); }
        else std::cout << tree.prod(l, r).value << '\n';
    }
}
