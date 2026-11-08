#include <iostream>
#include "dp_optimization/frontier_dp.hpp"
int main() {
    std::vector<std::pair<int,int>> edges{{0,1},{1,2},{2,3},{3,0}};
    std::cout << cp::frontier_hamiltonian_path_count(4, edges, 0, 3) << ' '
              << cp::frontier_hamiltonian_cycle_count(4, edges) << '\n';
}
