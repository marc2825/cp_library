#include "combinatorial_optimization/linear_program.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto r = linear_program<BigRational>({{1, 1}, {1, 0}, {0, 1}}, {4, 2, 3}, {3, 2});
    std::cout << lp_status_name(r.status) << " " << r.value << "\n";
    for (std::size_t i = 0; i < r.primal.size(); i++)
        std::cout << (i ? " " : "") << r.primal[i];
    std::cout << "\n";
}
