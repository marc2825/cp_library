#include <iostream>
#include <vector>
#include "number_theory_convolution/multiple_mobius_transform.hpp"
int main() {
    std::vector<long long> f{0, 21, 12, 9, 4, 5, 6};
    cp::multiple_mobius_transform(f);
    for (std::size_t i = 1; i < f.size(); ++i) std::cout << f[i] << ' ';
    std::cout << '\n';
}
