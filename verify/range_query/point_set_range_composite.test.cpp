#define PROBLEM "https://judge.yosupo.jp/problem/point_set_range_composite"
#include "range_query/segtree.hpp"
#include "test_values.hpp"
#include <iostream>
#include <vector>
using namespace range_test;
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q;
    std::vector<Affine> a(n); for (auto& f : a) std::cin >> f.a >> f.b;
    cp::Segtree<Affine, concat_affine, identity> tree(a);
    while (q--) {
        int t, l; std::cin >> t >> l;
        if (t == 0) { Affine f; std::cin >> f.a >> f.b; tree.set(l, f); }
        else { int r; long long x; std::cin >> r >> x; std::cout << range_test::apply(tree.prod(l, r), x) << '\n'; }
    }
}
