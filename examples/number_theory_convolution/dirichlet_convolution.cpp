#include <iostream>
#include <vector>
#include "number_theory_convolution/dirichlet_convolution.hpp"
int main() {
    std::vector<long long> one(7, 1); one[0] = 0;
    const auto count = cp::dirichlet_convolution(one, one); // 約数個数 tau(n)
    for (std::size_t i = 1; i < count.size(); ++i) std::cout << count[i] << ' ';
    std::cout << '\n';
}
