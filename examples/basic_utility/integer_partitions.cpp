#include <iostream>
#include "basic_utility/integer_partitions.hpp"
int main() {
    std::cout << cp::integer_partition_count(5) << '\n';
    int visited = 0;
    cp::enumerate_integer_partitions(5, [&](const std::vector<int>& parts) {
        for (std::size_t i = 0; i < parts.size(); ++i) std::cout << (i ? "+" : "") << parts[i];
        std::cout << '\n';
        return ++visited < 3;
    });
}
