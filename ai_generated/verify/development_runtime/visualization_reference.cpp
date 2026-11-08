#include <iostream>
#include <limits>
#include <string>
#include <tuple>
#include <vector>
#include "basic_utility/debug.hpp"
int main() {
    std::cout << cp::debug_string(std::make_tuple(std::string("state\n"), std::vector<int>{3, 1, 4}), {4, 2, 12}) << '\n';
    std::cout << cp::debug_string(std::vector<std::vector<int>>{{1, 2}, {3}}, {0, 32, 256}) << '\n';
    std::cout << cp::debug_string(std::numeric_limits<__int128_t>::min()) << '\n';
    std::cout << cp::debug_string(std::string("\1\n\"\\")) << '\n';
    std::cout << cp::debug_string(std::vector<int>{1, 2}, {8, 0, 256}) << '\n';
    std::cout << cp::debug_string(std::string("abcdef"), {8, 32, 3}) << '\n';
}
