#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include "basic_utility/debug.hpp"
int main(int argc, char** argv) {
    int n;
    if (!(std::cin >> n) || n < 1) return 1;
    std::vector<int> values(n);
    for (auto& value : values) if (!(std::cin >> value)) return 1;
    const auto index = argc > 1 && std::string(argv[1]) == "best"
        ? std::max_element(values.begin(), values.end()) - values.begin() : 0;
    CP_DEBUG(values, index);
    std::cout << index << '\n';
}
