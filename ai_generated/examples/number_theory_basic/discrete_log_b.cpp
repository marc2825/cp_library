#include <iostream>
#include "number_theory_basic/discrete_log.hpp"
int main() {
    std::cout << *cp::discrete_log(8,6,10) << '\n';
}
