#include <iostream>
#include "number_theory_convolution/divisor_count_sum.hpp"
int main() {
    std::cout << cp::optimization_to_string(cp::divisor_count_sum(10)) << '\n';
}
