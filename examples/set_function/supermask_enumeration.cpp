#include <cstdint>
#include <iostream>
#include "set_function/supermask_enumeration.hpp"
int main() {
    cp::for_each_supermask(2, 4, [](std::uint64_t s) { std::cout << s << ' '; });
    std::cout << '\n';
}
