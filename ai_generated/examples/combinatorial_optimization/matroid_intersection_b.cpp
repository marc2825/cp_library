#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/matroid_intersection.hpp"

int main() {
    auto first = [](const std::vector<int>& s) { return s.size() <= 2; };
    auto second = [](const std::vector<int>& s) { int even = 0; for (int x : s) even += x % 2 == 0; return even <= 1; };
    std::cout << cp::matroid_intersection(4, first, second).selected.size() << "\n";
}
