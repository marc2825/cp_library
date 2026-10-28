#include "computational_geometry/robust_predicates.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::cout << robust::orientation(exact::Point{0, 0}, exact::Point{4, 0}, exact::Point{0, 3}) << " "
              << robust::incircle(exact::Point{0, 0}, exact::Point{4, 0}, exact::Point{0, 3},
                                  exact::Point{1, 1})
              << "\n";
}
