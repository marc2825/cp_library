#include <iostream>
#include "combinatorics/shift_sample_points.hpp"
int main() {
    auto y=cp::shift_sample_points(std::vector<int>{0,1,4},3,3);
    std::cout << y[0] << ' ' << y[1] << ' ' << y[2] << '\n';
}
