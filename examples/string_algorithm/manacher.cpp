#include <iostream>
#include "string_algorithm/manacher.hpp"
int main() {
    auto radii = cp::manacher("abbaaba");
    for (int x : radii.odd) std::cout << x << ' ';
    std::cout << '\n';
    for (int x : radii.even) std::cout << x << ' ';
    std::cout << '\n';
    std::cout << radii.is_palindrome(0, 4) << ' ' << radii.is_palindrome(0, 7) << '\n';
}
