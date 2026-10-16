#define PROBLEM "https://judge.yosupo.jp/problem/rectangle_sum"
#include <iostream>
#include <vector>
#include "offline_query/sweep_line.hpp"

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    int n, q;
    std::cin >> n >> q;
    std::vector<cp::WeightedPoint<>> points(n);
    for (auto& p : points) std::cin >> p.x >> p.y >> p.weight;
    std::vector<cp::Rectangle<>> rectangles(q);
    for (auto& r : rectangles) std::cin >> r.left >> r.down >> r.right >> r.up;
    for (auto x : cp::static_rectangle_sum(std::move(points), rectangles)) std::cout << x << '\n';
}
