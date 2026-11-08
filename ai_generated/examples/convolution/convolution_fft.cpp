#include <complex>
#include <iomanip>
#include <iostream>
#include <vector>
#include "convolution/convolution_fft.hpp"
int main() {
    std::cout << std::fixed << std::setprecision(3);
    for (auto x : cp::convolution_fft(std::vector<double>{0.5, -1}, std::vector<double>{2, 3}))
        std::cout << x << ' ';
    std::cout << '\n';
    using C = std::complex<double>;
    for (auto x : cp::convolution_fft(std::vector<C>{{1, 1}, {2, 0}}, std::vector<C>{{0, 1}}))
        std::cout << x.real() << ',' << x.imag() << ' ';
    std::cout << '\n';
}
