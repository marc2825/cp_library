#include <iostream>
#include "basic_utility/barrett_reduction.hpp"
int main() {
    cp::BarrettReduction reducer(1000000007);
    std::cout << reducer.reduce(1000000009ULL) << " " << reducer.multiply(1000000006, 1000000006) << "\n";
}
