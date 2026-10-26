#include <iostream>
#include "number_theory_basic/mod_kth_root.hpp"

int main() {
    const auto roots = cp::mod_kth_roots(4, 4, 17);
    for (auto x : roots.enumerate()) std::cout << x << ' ';
    std::cout << '\n';
}
