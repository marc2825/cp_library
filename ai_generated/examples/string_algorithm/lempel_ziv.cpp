#include "string_algorithm/lempel_ziv.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto f = cp::lempel_ziv_factorization("aaaaabaaaaab");
    for (auto x : f)
        std::cout << x.source << ' ' << x.length << ' ' << int(x.literal) << '\n';
    std::cout << (cp::restore_lz77(f) == "aaaaabaaaaab") << '\n';
}
