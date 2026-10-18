#include <iostream>
#include "basic_utility/stern_brocot_tree.hpp"
int main() {
    auto f=cp::stern_brocot_fraction(cp::stern_brocot_path(12,8));
    std::cout << f.numerator << "/" << f.denominator << "\n";
}
