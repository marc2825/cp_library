#include "fps/polynomial_extended_gcd.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    auto r = cp::polynomial_extended_gcd({2, -3, 1}, {-1, 1});
    print(r.gcd);
    print(*cp::polynomial_mod_inverse({0, 1}, {1, 0, 1}));
}
