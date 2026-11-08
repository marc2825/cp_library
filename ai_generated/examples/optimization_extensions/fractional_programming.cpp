#include "combinatorial_optimization/fractional_programming.hpp"
#include <iostream>
using namespace cp;
int main() {
    std::vector<FractionalPoint> points{{5, 2, {}}, {7, 3, {}}, {9, 4, {}}};
    auto oracle = [&](long double lambda) -> std::optional<FractionalPoint> {
        auto best = points[0];
        for (auto p : points)
            if (p.numerator - lambda * p.denominator > best.numerator - lambda * best.denominator)
                best = p;
        return best;
    };
    FractionalOptions options;
    options.denominator_lower_bound = 2;
    std::cout << fractional_dinkelbach(oracle, 0, options).ratio << " "
              << fractional_bisection(oracle, 0, 5, options).ratio << "\n";
}
