#include <iostream>
#include "combinatorics/bernoulli_numbers.hpp"
int main() {
    auto b=cp::bernoulli_numbers(2,101); std::cout << b[1] << ' ' << b[2] << '\n';
}
