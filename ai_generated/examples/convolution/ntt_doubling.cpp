#include "convolution/ntt_doubling.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::vector<int> a{1, 2};
    cp::ntt<17, 3>(a);
    cp::ntt_doubling<17, 3>(a);
    print(a);
}
