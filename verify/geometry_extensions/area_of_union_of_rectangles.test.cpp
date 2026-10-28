#define PROBLEM "https://judge.yosupo.jp/problem/area_of_union_of_rectangles"
#include "computational_geometry/rectangle_union.hpp"
#include <iostream>
int main() {
    int n;
    std::cin >> n;
    std::vector<cp::geometry::exact::Rectangle> rectangles(n);
    for (auto &r : rectangles)
        std::cin >> r.x1 >> r.y1 >> r.x2 >> r.y2;
    std::cout << cp::geometry::exact::rectangle_union_area(rectangles) << '\n';
}
