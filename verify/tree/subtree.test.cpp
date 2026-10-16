#define PROBLEM "https://atcoder.jp/contests/dp/tasks/dp_v"
#include <iostream>
#include "tree/rerooting.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n; long long mod; std::cin >> n >> mod; cp::Graph<> g(n);
    for (int i = 1, u, v; i < n; ++i) { std::cin >> u >> v; g.add_edge(u - 1, v - 1); }
    auto dp = cp::rerooting(g, 1LL,
        [=](long long a, long long b) { return a * b % mod; },
        [](long long a, int) { return a; },
        [=](long long a, const cp::Edge<long long>&) { return (a + 1) % mod; });
    for (auto value : dp) std::cout << value << '\n';
}
