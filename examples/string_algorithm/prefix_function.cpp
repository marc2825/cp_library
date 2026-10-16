#include <iostream>
#include "string_algorithm/prefix_function.hpp"
int main() {
    for (int x : cp::prefix_function("ababaca")) std::cout << x << ' ';
    std::cout << '\n';
}
