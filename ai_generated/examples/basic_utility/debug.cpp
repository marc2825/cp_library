#include <iostream>
#include <string>
#include <tuple>
#include <vector>
#include "basic_utility/debug.hpp"
int main() {
    std::vector<int> values{3, 1, 4};
    CP_DEBUG(values);
    std::cout << cp::debug_string(std::make_tuple("state", values, true)) << '\n';
    std::cout << cp::debug_string(values, {8, 2, 256}) << '\n';
}
