#include <iostream>
#include <vector>
#include "../../string_algorithm/palindrome_partition.hpp"

int main() {
    auto result = cp::palindrome_partition("abacdc");
    std::cout << result.prefix_minimum.back() << " " << result.intervals.size() << "\n";
}
