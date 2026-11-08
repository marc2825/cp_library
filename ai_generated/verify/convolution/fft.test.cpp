#define PROBLEM "https://atcoder.jp/contests/atc001/tasks/fft_c"
#include <cmath>
#include <complex>
#include <iostream>
#include <vector>
#include "convolution/fft.hpp"
int main() {
    int n; std::cin >> n;
    std::vector<long long> a(n+1), b(n+1);
    for (int i = 1; i <= n; ++i) std::cin >> a[i] >> b[i];
    int padded = 1; while (padded < 2*n+1) padded *= 2;
    std::vector<std::complex<double>> x(a.begin(), a.end()), y(b.begin(), b.end());
    x.resize(padded); y.resize(padded);
    cp::fft(x); cp::fft(y);
    for (int k = 0; k < padded; ++k) x[k] *= y[k];
    cp::fft(x, true);
    for (int k = 1; k <= 2*n; ++k) std::cout << std::llround(x[k].real()) << '\n';
}
