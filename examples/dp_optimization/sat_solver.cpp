#include <iostream>
#include "dp_optimization/sat_solver.hpp"
int main() {
    cp::SatSolver sat(3);
    sat.add_clause({1,2}); sat.add_clause({-1,3});
    sat.add_clause({-2,3}); sat.add_clause({-3,2});
    std::cout << bool(sat.solve()) << ' ' << bool(sat.solve({-3})) << '\n';
}
