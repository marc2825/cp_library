#include <iostream>
#include "string_algorithm/suffix_array.hpp"
int main() {
    for (int pos : cp::suffix_array("banana")) std::cout << pos << ' ';
    std::cout << '\n';
}
