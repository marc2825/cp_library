#include <iostream>
#include "dp_optimization/dynamic_finite_state_dp.hpp"
int main() {
    using Matrix = cp::DynamicFiniteStateDP::Matrix;
    cp::DynamicFiniteStateDP dp(2, {Matrix{{0,3},{2,1}}, Matrix{{1,0},{4,2}}});
    std::cout << *dp.all_product()[0][1] << ' ';
    dp.set(0, Matrix{{2,5},{2,1}});
    std::cout << *dp.all_product()[0][1] << '\n';
    auto path = dp.restore(0,2,0,1);
    for (std::size_t i = 0; i < path->states.size(); ++i) std::cout << (i ? " " : "") << path->states[i];
    std::cout << '\n';
}
