#include <iostream>
#include "number_theory_convolution/prime_counting.hpp"
int main() {
    cp::PrimeCounting p(100); std::cout << p.count() << ' ' << p.count_quotient(10) << '\n';
}
