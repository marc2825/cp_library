#include <iostream>
#include "combinatorics/hook_length.hpp"
int main() {
    const cp::YoungShape shape{3, 2};
    std::cout << cp::standard_young_tableaux_count(shape) << ' '
              << cp::standard_young_tableaux_count_mod(shape, 4) << ' '
              << cp::semistandard_young_tableaux_count(shape, 3) << '\n';
}
