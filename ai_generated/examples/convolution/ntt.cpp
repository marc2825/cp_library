#include <iostream>
#include <vector>
#include "convolution/ntt.hpp"
int main() {
    std::vector<int> a{1, 2, 3, 4};
    cp::ntt<17, 3>(a);
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
    cp::ntt<17, 3>(a, true);
    for (int x : a) std::cout << x << ' ';
    std::cout << '\n';
}
