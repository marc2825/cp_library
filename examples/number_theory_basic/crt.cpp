#include <iostream>
#include <vector>
#include "number_theory_basic/crt.hpp"

int main() {
    auto result=cp::crt({2,8},{6,9});
    if (result) std::cout << result->remainder << " " << result->modulus << "\n";
}
