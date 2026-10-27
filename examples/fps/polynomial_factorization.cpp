#include "fps/polynomial_factorization.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    auto f = cp::polynomial_factorization<3>({2, 1, 0, 1, 2});
    std::cout << f.leading_coefficient << "\n";
    for (const auto &part : f.factors) {
        print(part.polynomial);
        std::cout << part.multiplicity << "\n";
    }
}
