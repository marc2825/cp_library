#include <iostream>
#include <tuple>
#include <vector>
#include "basic_utility/seeded_hash.hpp"
int main() {
    cp::SeededHash hash(42), replay(42);
    auto key = std::make_tuple(-7, std::vector<int>{1, 2, 3});
    std::cout << hash.seed() << " " << (hash.hash64(key) == replay.hash64(key)) << "\n";
}
