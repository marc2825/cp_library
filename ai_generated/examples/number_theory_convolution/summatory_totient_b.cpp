#include <iostream>
#include "number_theory_convolution/summatory_totient.hpp"
int main() {
    cp::SummatoryTotient sum(16); std::cout << cp::optimization_to_string(sum(10)) << '\n';
}
