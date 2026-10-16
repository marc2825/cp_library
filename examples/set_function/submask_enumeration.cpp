#include <cstdint>
#include <iostream>
#include "set_function/submask_enumeration.hpp"
int main() {
    cp::for_each_submask(10, [](std::uint64_t s) { std::cout << s << ' '; });
    std::cout << '\n';
    cp::for_each_submask(10, [](std::uint64_t s) { std::cout << s << ' '; }, false);
    std::cout << '\n';
}
