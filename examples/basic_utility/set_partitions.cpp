#include <iostream>
#include "basic_utility/set_partitions.hpp"
int main() {
    std::cout << cp::set_partition_count(3) << '\n';
    cp::enumerate_set_partitions(3, [](const std::vector<int>& rgs) {
        for (int block : rgs) std::cout << block;
        std::cout << '\n';
        return true;
    });
}
