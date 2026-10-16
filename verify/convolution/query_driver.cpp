#include <complex>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "convolution/all.hpp"
int main() {
    std::cout << std::setprecision(21);
    char kind; int n, m;
    while (std::cin >> kind >> n >> m) {
        std::vector<long long> a(n), b(m);
        for (auto& x : a) std::cin >> x;
        for (auto& x : b) std::cin >> x;
        try {
            if (kind == 'M' || kind == 'S') {
                const auto c = kind == 'M' ? cp::convolution(a, b) : cp::convolution<17, 3>(a, b);
                std::cout << c.size(); for (auto x : c) std::cout << ' ' << x;
            } else if (kind == 'L') {
                const auto c = cp::convolution_ll(a, b);
                std::cout << c.size(); for (auto x : c) std::cout << ' ' << x;
            } else {
                const auto c = cp::convolution_fft(std::vector<long double>(a.begin(), a.end()), std::vector<long double>(b.begin(), b.end()));
                std::cout << c.size(); for (auto x : c) std::cout << ' ' << x;
            }
        } catch (const std::length_error&) { std::cout << "LENGTH"; }
          catch (const std::overflow_error&) { std::cout << "OVERFLOW"; }
        std::cout << '\n';
    }
}
