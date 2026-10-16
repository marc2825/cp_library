#include <iostream>
#include "string_algorithm/kmp.hpp"
int main() {
    for (int pos : cp::kmp_search("abababa", "aba")) std::cout << pos << ' ';
    std::cout << '\n';
    for (int pos : cp::kmp_search("ab", "")) std::cout << pos << ' ';
    std::cout << '\n';
}
