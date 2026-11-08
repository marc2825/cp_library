#include <iostream>
#include "dp_optimization/exact_cover.hpp"
int main() {
    cp::ExactCover cover(3);
    cover.add_row({0,1}); cover.add_row({2});
    cover.add_row({0}); cover.add_row({1,2});
    std::cout << cover.enumerate([](const auto&) { return true; });
    auto solution = cover.solve();
    for (int row : *solution) std::cout << ' ' << row;
    std::cout << '\n';
}
