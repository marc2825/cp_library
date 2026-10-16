#define PROBLEM "https://cses.fi/problemset/task/1750"
#include <iostream>
#include <vector>
#include "tree/binary_lifting.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n, q; std::cin >> n >> q; std::vector<int> to(n);
    for (auto& v : to) { std::cin >> v; --v; }
    cp::BinaryLifting jump(to, 1000000000);
    while (q--) { int v; std::uint64_t k; std::cin >> v >> k; std::cout << jump.jump(v - 1, k) + 1 << '\n'; }
}
