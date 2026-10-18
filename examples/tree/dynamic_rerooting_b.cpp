#include <iostream>
#include "tree/dynamic_rerooting.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    cp::DynamicTreeDistanceSum<> dp(g,{1,1,1});
    std::cout << dp.query(0) << ' '; dp.set_vertex(2,2); std::cout << dp.query(1) << '\n';
}
