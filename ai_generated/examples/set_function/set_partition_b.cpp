#include <iostream>
#include <vector>
#include "../../set_function/set_partition.hpp"

int main() {
    auto a = cp::set_partition_by_blocks(std::vector<int>(8, 1));
    std::cout << a[2][7] << "\n";
}
