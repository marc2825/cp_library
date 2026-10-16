#include <iostream>
#include <optional>
#include <vector>
#include "dp_optimization/subset_dp.hpp"
int main() {
    std::vector<std::vector<std::optional<long long>>> cost{
        {std::nullopt,2,8}, {4,std::nullopt,3}, {1,6,std::nullopt}};
    auto tour = cp::minimum_hamiltonian_cycle(cost);
    if (tour.exists) {
        std::cout << tour.cost << '\n';
        for (int v : tour.vertices) std::cout << v << ' ';
        std::cout << '\n';
    }
    std::vector<long long> f{1,2,3,4};
    cp::subset_zeta_transform(f);
    for (auto x : f) std::cout << x << ' ';
    std::cout << '\n';
}
