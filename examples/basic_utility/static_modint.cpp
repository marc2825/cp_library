#include <iostream>
#include "basic_utility/static_modint.hpp"
int main() {
    using Mint = cp::StaticModInt<12>;
    std::cout << Mint(-5) << " " << Mint(5).inv() << " " << Mint(6).try_inv().has_value() << "\n";
}
