#define PROBLEM "https://judge.yosupo.jp/problem/staticrmq"
#include "static_query/sparse_table.hpp"
#include "test_values.hpp"
#include <iostream>
#include <vector>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q;
    std::vector<int> a(n); for (auto& x : a) std::cin >> x;
    cp::SparseTable<int, static_test::minimum, static_test::infinity> table(a);
    while (q--) { int l, r; std::cin >> l >> r; std::cout << table.prod(l, r) << '\n'; }
}
