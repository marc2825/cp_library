#include <iostream>
#include "number_theory_basic/sum_two_squares.hpp"

int main() {
    for (auto [x, y] : cp::sum_two_squares(65)) std::cout << x << ' ' << y << '\n';
}
