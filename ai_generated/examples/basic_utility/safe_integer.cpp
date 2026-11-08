#include <iostream>
#include <limits>
#include "basic_utility/safe_integer.hpp"
int main() {
    const auto high = std::numeric_limits<long long>::max();
    std::cout << cp::checked_add(high, 1LL).has_value() << " " << cp::saturating_add(high, 1LL) << "\n";
    std::cout << cp::floor_div(-7, 3) << " " << cp::ceil_div(-7, 3) << "\n";
    std::cout << cp::infinity_add(7, 8, 10) << " " << cp::multiply_leq(7, 8, 50) << "\n";
}
