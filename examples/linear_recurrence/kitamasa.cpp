#include <iostream>
#include "linear_recurrence/kitamasa.hpp"
int main() {
    std::cout << cp::kitamasa({0, 1}, {1, 1}, 10) << '\n';
    for (int x : cp::kitamasa_coefficients<17>({1, 1}, 10)) std::cout << x << ' ';
    std::cout << '\n';
}
