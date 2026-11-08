#include <array>
#include <iostream>
#include "tree/tree_dp.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3);
    // {根を白にする場合の数, 根を黒にする場合の数}。隣り合う黒を禁止する。
    using S = std::array<long long, 2>;
    auto dp = cp::tree_dp(g, S{1, 1},
        [](S a, S b) { return S{a[0] * b[0], a[1] * b[1]}; },
        [](S a, int) { return a; },
        [](S a, const cp::Edge<long long>&) { return S{a[0] + a[1], a[0]}; });
    std::cout << dp[0][0] + dp[0][1] << '\n';
}
