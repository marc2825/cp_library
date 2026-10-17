#include <iostream>
#include <vector>
#include "dp_optimization/slope_trick.hpp"

int main() {
    cp::SlopeTrick f; f.add_abs(1); f.add_abs(5); f.shift(-1,2);
    auto [l,r]=f.argmin();
    std::cout << f.minimum() << " " << *l << " " << *r << "\n";
}
