#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DSL_2_G"
#include "range_query/range_add_range_sum_fenwick_tree.hpp"
#include <iostream>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q;
    cp::RangeAddRangeSumFenwickTree<long long> tree(n);
    while (q--) {
        int t, s, e; std::cin >> t >> s >> e;
        if (t == 0) { long long x; std::cin >> x; tree.add(s - 1, e, x); }
        else std::cout << tree.sum(s - 1, e) << '\n';
    }
}
