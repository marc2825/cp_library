#include <complex>
#include <iomanip>
#include <iostream>
#include <vector>
#include "convolution/fft.hpp"
int main() {
    auto spectrum = cp::fft_real(std::vector<double>{1, 2, 3, 4});
    cp::fft(spectrum, true);
    std::cout << std::fixed << std::setprecision(3);
    for (auto x : spectrum) std::cout << x.real() << ' ';
    std::cout << '\n';
    std::vector<std::complex<long double>> z{{1, 1}, {2, -1}};
    cp::fft(z);
    for (auto x : z) std::cout << x.real() << ',' << x.imag() << ' ';
    std::cout << '\n';
}
