#include "combinatorial_optimization/matroid_union.hpp"
#include <iostream>
using namespace cp;
int main() {
    std::vector<int> groups{0, 0, 1, 1, 1};
    std::vector<MatroidOracle> oracles;
    for (auto capacities : std::vector<std::vector<int>>{{1, 2}, {1, 1}})
        oracles.emplace_back([groups, capacities](const std::vector<int> &s) {
            auto left = capacities;
            for (int e : s)
                if (--left[groups[e]] < 0)
                    return false;
            return true;
        });
    auto r = matroid_partition(5, oracles);
    std::cout << r.cardinality << " " << r.complete << "\n";
}
