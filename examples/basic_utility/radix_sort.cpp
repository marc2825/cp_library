#include <iostream>
#include <utility>
#include <vector>
#include "basic_utility/radix_sort.hpp"
int main() {
    std::vector<std::pair<int, char>> items{{3, 'a'}, {-1, 'b'}, {3, 'c'}, {0, 'd'}};
    cp::radix_sort_by(items, [](const auto& item) { return item.first; });
    for (const auto& item : items) std::cout << item.first << ":" << item.second << "\n";
}
