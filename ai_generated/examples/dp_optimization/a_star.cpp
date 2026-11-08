#include <iostream>
#include "dp_optimization/a_star.hpp"
int main() {
    std::vector<std::vector<cp::StateSearchEdge<int>>> edges{
        {{1,5,0},{2,1,1}}, {{3,10,2}}, {{1,1,3}}, {}};
    std::vector<int> h{0,0,11,0};
    auto result = cp::a_star(0, [&](int v) { return edges[v]; },
        [](int v) { return v == 3; }, [&](int v) { return h[v]; });
    std::cout << result->cost;
    for (int v : result->states) std::cout << ' ' << v;
    std::cout << '\n';
}
