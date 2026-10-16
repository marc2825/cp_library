#define PROBLEM "https://judge.yosupo.jp/problem/predecessor_problem"
#include "ordered_structure/ordered_associative.hpp"
#include <iostream>
#include <set>
#include <string>
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::string s; std::cin >> n >> q >> s;
    std::set<int> values; for (int i = 0; i < n; ++i) if (s[i] == '1') values.insert(i);
    while (q--) {
        int type, x; std::cin >> type >> x;
        if (type == 0) values.insert(x);
        else if (type == 1) cp::erase_one(values, x);
        else if (type == 2) std::cout << values.count(x) << '\n';
        else {
            auto it = type == 3 ? cp::successor(values, x, true) : cp::predecessor(values, x, true);
            std::cout << (it == values.end() ? -1 : *it) << '\n';
        }
    }
}
