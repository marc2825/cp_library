#include <iostream>
#include <vector>
#include "offline_query/sweep_line.hpp"

int main() {
    std::vector<cp::WeightedPoint<>> points{{1, 2, 4}, {3, 2, 7}, {1, 2, -1}};
    std::vector<cp::Rectangle<>> rectangles{{1, 2, 3, 3}, {0, 0, 4, 3}, {3, 2, 3, 3}};
    auto answer = cp::static_rectangle_sum(points, rectangles);
    for (auto x : answer) std::cout << x << '\n'; // 3, 10, 0
}
