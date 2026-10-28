#include "numerical_analysis/stable_arithmetic.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    KahanSum kahan;
    CompensatedSum compensated;
    for (auto x : {1e30L, 1.L, -1e30L}) {
        kahan.add(x);
        compensated.add(x);
    }
    std::cout << kahan.value() << " " << compensated.value() << "\n";
}
