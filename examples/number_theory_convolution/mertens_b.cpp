#include <iostream>
#include "number_theory_convolution/mertens.hpp"
int main() {
    cp::MertensFunction m(4); std::cout << cp::optimization_to_string(m(10)) << '\n';
}
