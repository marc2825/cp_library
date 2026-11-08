#include <iostream>
#include "string_algorithm/z_algorithm.hpp"
int main() {
    for (int x : cp::z_algorithm("aabcaabxaaaz")) std::cout << x << ' ';
    std::cout << '\n';
}
