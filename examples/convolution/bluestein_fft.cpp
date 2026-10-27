#include "convolution/bluestein_fft.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}
int main() {
    std::vector<std::complex<double>> a{{1, 0}, {2, 0}, {3, 0}};
    cp::bluestein_fft(a);
    for (auto x : a)
        std::cout << std::llround(x.real() * 1000000) << " " << std::llround(x.imag() * 1000000) << "\n";
}
