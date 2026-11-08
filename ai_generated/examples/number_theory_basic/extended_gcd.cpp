#include <iostream>
#include "number_theory_basic/extended_gcd.hpp"
int main() {
    auto result = cp::extended_gcd(-30, 18);
    std::cout << result.gcd << ' ' << result.x << ' ' << result.y << '\n';
    std::cout << -30 * result.x + 18 * result.y << '\n';
}
