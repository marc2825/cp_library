#include <iostream>
#include "combinatorics/eulerian_numbers.hpp"
int main() {
    auto a=cp::eulerian_numbers(4); std::cout << a[4][1] << '\n';
}
