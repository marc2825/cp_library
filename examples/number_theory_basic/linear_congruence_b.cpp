#include <iostream>
#include "number_theory_basic/linear_congruence.hpp"
int main() {
    auto r=cp::linear_congruence(6,8,14);
    std::cout << r->first << ' ' << r->step << ' ' << r->count << '\n';
}
