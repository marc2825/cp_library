#include <iostream>
#include "game_theory/nim_product.hpp"
int main() {
    std::cout << cp::nim_product(2, 3) << ' ' << cp::nim_product(16, 16) << '\n';
    std::cout << cp::nim_product(UINT64_C(1) << 32, UINT64_C(1) << 32) << '\n';
}
