#include <iostream>
#include "string_algorithm/lyndon_factorization.hpp"
int main() {
    for (auto [l, r] : cp::lyndon_factorization("banana")) std::cout << l << ' ' << r << '\n';
}
