#include <iostream>
#include <vector>
#include "number_theory_convolution/divisor_zeta_transform.hpp"
int main() {
    std::vector<long long> f{99, 1, 2, 3, 4, 5, 6};
    cp::divisor_zeta_transform(f);
    for (std::size_t i = 1; i < f.size(); ++i) std::cout << f[i] << ' ';
    std::cout << '\n' << f[0] << '\n';
}
