#define PROBLEM "https://judge.yosupo.jp/problem/static_range_inversions_query"
#include <iostream>
#include <vector>
#include "offline_query/mo.hpp"
#include "ordered_structure/coordinate_compression.hpp"
#include "range_query/fenwick_tree.hpp"

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    int n, q;
    std::cin >> n >> q;
    std::vector<long long> input(n);
    for (auto& x : input) std::cin >> x;
    cp::CoordinateCompression<long long> comp(input);
    std::vector<int> a(n);
    for (int i = 0; i < n; ++i) a[i] = *comp.index(input[i]);
    cp::Mo mo(n);
    for (int i = 0, l, r; i < q; ++i) { std::cin >> l >> r; mo.add_query(l, r); }
    cp::FenwickTree<int> frequency(comp.size());
    int length = 0;
    long long inversions = 0;
    std::vector<long long> answer(q);
    mo.run([&](int i) { inversions += frequency.sum(a[i]); frequency.add(a[i], 1); ++length; },
           [&](int i) { inversions += length - frequency.sum(a[i] + 1); frequency.add(a[i], 1); ++length; },
           [&](int i) { frequency.add(a[i], -1); --length; inversions -= frequency.sum(a[i]); },
           [&](int i) { frequency.add(a[i], -1); --length; inversions -= length - frequency.sum(a[i] + 1); },
           [&](int id) { answer[id] = inversions; });
    for (auto x : answer) std::cout << x << '\n';
}
