#include <iostream>
#include "basic_utility/dynamic_modint.hpp"
int main() {
    using A = cp::DynamicModInt<0>;
    using B = cp::DynamicModInt<1>;
    A::set_mod(12); B::set_mod(7);
    A a = 5;
    std::cout << a.pow(3) << " " << B(5).pow(3) << "\n";
    A::set_mod(11);
    try { std::cout << a.val(); }
    catch (const std::logic_error&) { std::cout << "stale\n"; }
}
