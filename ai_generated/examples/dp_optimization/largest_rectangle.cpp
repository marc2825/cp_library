#include <iostream>
#include "dp_optimization/largest_rectangle.hpp"
int main() {
    auto histogram = cp::largest_rectangle_histogram({2,1,4,5,1,3,3});
    std::cout << histogram.area << ' ' << histogram.left << ' ' << histogram.right << '\n';
    auto grid = cp::largest_rectangle_grid({{false,false,true}, {false,false,false}});
    std::cout << grid.area << ' ' << grid.top << ' ' << grid.left << ' ' << grid.bottom << ' ' << grid.right << '\n';
}
