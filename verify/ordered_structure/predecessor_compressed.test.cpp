#define PROBLEM "https://judge.yosupo.jp/problem/predecessor_problem"
#include "ordered_structure/compressed_order_statistics.hpp"
#include <iostream>
#include <string>
#include <utility>
#include <vector>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::string s; std::cin >> n >> q >> s;
    std::vector<int> universe; for (int i = 0; i < n; ++i) if (s[i] == '1') universe.push_back(i);
    std::vector<std::pair<int, int>> queries(q);
    for (auto& [type, x] : queries) { std::cin >> type >> x; if (type == 0) universe.push_back(x); }
    cp::CompressedOrderStatistics<int> values(universe);
    for (int i = 0; i < n; ++i) if (s[i] == '1') values.insert(i);
    for (auto [type, x] : queries) {
        if (type == 0) { if (values.count(x) == 0) values.insert(x); } // set semantics
        else if (type == 1) values.erase_all(x);
        else if (type == 2) std::cout << values.count(x) << '\n';
        else std::cout << (type == 3 ? values.successor(x, true) : values.predecessor(x, true)).value_or(-1) << '\n';
    }
}
