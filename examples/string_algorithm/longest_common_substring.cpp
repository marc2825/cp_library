#include <iostream>
#include "string_algorithm/longest_common_substring.hpp"
int main() {
    auto r = cp::longest_common_substring("banana", "ananas");
    std::cout << r.first_left << ' ' << r.first_right() << ' ' << r.second_left << ' ' << r.second_right() << '\n';
}
