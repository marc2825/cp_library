#include <iostream>
#include "number_theory_basic/pell_equation.hpp"

int main() {
    const auto fundamental = cp::pell_fundamental(2);
    const auto third = cp::pell_solution_power(2, *fundamental, 3);
    std::cout << fundamental->x << ' ' << fundamental->y << '\n';
    std::cout << third.x << ' ' << third.y << '\n';
}
