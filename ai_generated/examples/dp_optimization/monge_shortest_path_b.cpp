#include <iostream>
#include "dp_optimization/monge_shortest_path.hpp"
int main() {
    auto r=cp::monge_shortest_path(4,[](int i,int j){return 1LL*(j-i)*(j-i);});
    std::cout << r.distance[4] << ' ' << r.path_to(4).size() << '\n';
}
