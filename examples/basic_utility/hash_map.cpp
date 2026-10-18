#include <iostream>
#include <utility>
#include "basic_utility/hash_map.hpp"
int main() {
    cp::HashMap<std::pair<int, int>, int> count(cp::SeededHash(42));
    count[{2, 3}] = 7;
    auto result = count.try_emplace({2, 3}, 9);
    std::cout << *result.first << " " << result.second << "\n";
    count.erase({2, 3});
    std::cout << count.contains({2, 3}) << "\n";
}
