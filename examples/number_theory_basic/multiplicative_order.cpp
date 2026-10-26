#include <iostream>
#include "number_theory_basic/multiplicative_order.hpp"

int main() {
    std::cout << cp::carmichael_lambda(360) << ' '
              << *cp::multiplicative_order(7, 40) << '\n';
}
