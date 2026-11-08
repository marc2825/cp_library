#include <iostream>
#include "number_theory_basic/discrete_log_table.hpp"

int main() {
    cp::DiscreteLogTable logarithm(2, 12);
    for (auto b : {1, 2, 4, 8, 3}) {
        const auto x = logarithm.query(b);
        std::cout << (x ? static_cast<long long>(*x) : -1) << ' ';
    }
    std::cout << '\n';
}
