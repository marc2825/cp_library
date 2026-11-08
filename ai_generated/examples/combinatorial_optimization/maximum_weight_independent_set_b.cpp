#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/maximum_weight_independent_set.hpp"
#include "../../combinatorial_optimization/partition_matroid.hpp"

int main() {
    cp::PartitionMatroid matroid({0, 0, 1}, {1, 1});
    auto result = cp::maximum_weight_matroid_independent_set(std::vector<long long>{2, 5, 3}, matroid);
    std::cout << cp::optimization_to_string(result.weight) << "\n";
}
