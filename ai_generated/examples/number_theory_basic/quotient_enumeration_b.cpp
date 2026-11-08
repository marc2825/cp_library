#include <iostream>
#include "number_theory_basic/quotient_enumeration.hpp"
int main() {
    cp::for_each_quotient(10,[](auto l,auto r,auto q){if(q==2) std::cout << l << ' ' << r << ' ' << q << '\n';});
}
