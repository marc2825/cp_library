#include "combinatorial_optimization/integer_program.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto r = bounded_integer_program({{1, 1}}, {BigRational(7, 2)}, {3, 2}, {0, 0}, {4, 4});
    std::cout << r.value << "\n";
    for (std::size_t i = 0; i < r.solution.size(); i++)
        std::cout << (i ? " " : "") << r.solution[i];
    std::cout << "\n";
}
