#include "combinatorial_optimization/low_dimensional_lp.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto r = low_dimensional_lp<2>({{{1, 1}}, {{-1, 0}}, {{0, -1}}}, {4, 0, 0}, {{2, 1}});
    std::cout << lp_status_name(r.status) << " " << r.value << "\n";
}
