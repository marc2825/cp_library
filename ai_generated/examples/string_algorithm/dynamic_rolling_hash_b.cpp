#include <iostream>
#include <vector>
#include "../../string_algorithm/dynamic_rolling_hash.hpp"

int main() {
    cp::DynamicRollingHash text("abc");
    text.insert(1, 'x'); text.reverse(0, 4); text.set(0, 'z');
    std::cout << text.str() << "\n";
}
