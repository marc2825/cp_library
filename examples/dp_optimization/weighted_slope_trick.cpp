#include <iostream>
#include "dp_optimization/weighted_slope_trick.hpp"
int main() {
    cp::WeightedSlopeTrick f;
    f.add_abs(1,2); f.add_abs(5,3); f.shift(-1,2); f.reflect();
    auto [left,right] = f.argmin();
    std::cout << f.minimum() << ' ' << *left << ' ' << *right << ' ' << f.evaluate(0) << '\n';
}
