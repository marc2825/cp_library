#include <iostream>
#include "combinatorics/q_binomial.hpp"
int main() {
    cp::QBinomial q(7, 2);
    std::cout << q(8, 3) << ' ' << q.factorial(8) << ' ' << q.zero_period() << '\n';
}
