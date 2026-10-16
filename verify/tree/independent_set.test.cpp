#define PROBLEM "https://atcoder.jp/contests/dp/tasks/dp_p"
#include <array>
#include <iostream>
#include "tree/tree_dp.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; std::cin >> n; cp::Graph<> g(n);
    for (int i = 1, u, v; i < n; ++i) { std::cin >> u >> v; g.add_edge(u - 1, v - 1); }
    constexpr long long mod = 1000000007; using S = std::array<long long, 2>;
    auto dp = cp::tree_dp(g, S{1, 1},
        [](S a, S b) { return S{a[0] * b[0] % mod, a[1] * b[1] % mod}; },
        [](S a, int) { return a; },
        [](S a, const cp::Edge<long long>&) { return S{(a[0] + a[1]) % mod, a[0]}; });
    std::cout << (dp[0][0] + dp[0][1]) % mod << '\n';
}
