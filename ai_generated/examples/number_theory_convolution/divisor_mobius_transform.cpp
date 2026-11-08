#include <iostream>
#include <vector>
#include "number_theory_convolution/divisor_mobius_transform.hpp"
int main() {
    std::vector<long long> f{0, 1, 2, 3, 4, 5, 6}; // id(n)=n
    cp::divisor_mobius_transform(f); // phi = mu * id
    for (std::size_t i = 1; i < f.size(); ++i) std::cout << f[i] << ' ';
    std::cout << '\n';
}
