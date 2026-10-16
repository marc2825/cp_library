#define PROBLEM "https://judge.yosupo.jp/problem/ordered_set"
#include "ordered_structure/compressed_order_statistics.hpp"
#include <iostream>
#include <utility>
#include <vector>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q;
    std::vector<long long> initial(n); for (auto& x : initial) std::cin >> x;
    std::vector<long long> universe = initial;
    std::vector<std::pair<int, long long>> queries(q);
    for (auto& [type, x] : queries) { std::cin >> type >> x; if (type == 0) universe.push_back(x); }
    cp::CompressedOrderStatistics<long long> values(universe);
    for (auto x : initial) if (!values.count(x)) values.insert(x);
    for (auto [type, x] : queries) {
        if (type == 0) { if (!values.count(x)) values.insert(x); }
        else if (type == 1) values.erase_all(x);
        else if (type == 2) std::cout << values.kth(x - 1).value_or(-1) << '\n'; // judge uses 1-based ranks
        else if (type == 3) std::cout << values.count_less_equal(x) << '\n';
        else if (type == 4) std::cout << values.predecessor(x, true).value_or(-1) << '\n';
        else std::cout << values.successor(x, true).value_or(-1) << '\n';
    }
}
