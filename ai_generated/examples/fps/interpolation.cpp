#include <iostream>
#include <vector>
#include "fps/interpolation.hpp"

void print(const std::vector<int>& a) {
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
int main() {
    print(cp::polynomial_interpolation<17, 3>({0, 1, 2}, {1, 6, 0}));
}
