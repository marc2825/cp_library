#define PROBLEM "https://atcoder.jp/contests/arc033/tasks/arc033_3"
#include "ordered_structure/compressed_order_statistics.hpp"
#include <cassert>
#include <iostream>
#include <utility>
#include <vector>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int q; std::cin >> q;
    std::vector<std::pair<int, int>> queries(q); std::vector<int> universe;
    for (auto& [type, x] : queries) { std::cin >> type >> x; if (type == 1) universe.push_back(x); }
    cp::CompressedOrderStatistics<int> values(universe);
    for (auto [type, x] : queries) {
        if (type == 1) values.insert(x);
        else { auto value = values.kth(x - 1); assert(value); std::cout << *value << '\n'; values.erase(*value); }
    }
}
