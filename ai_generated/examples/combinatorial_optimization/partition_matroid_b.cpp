#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/partition_matroid.hpp"

int main() {
    cp::PartitionMatroid matroid({0, 0, 1}, {1, 2});
    std::cout << matroid.independent({0, 2}) << " " << matroid.independent({0, 1}) << "\n";
}
