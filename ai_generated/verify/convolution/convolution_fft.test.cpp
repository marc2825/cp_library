#define PROBLEM "https://atcoder.jp/contests/atc001/tasks/fft_c"
#include <cmath>
#include <complex>
#include <iostream>
#include <vector>
#include "convolution/convolution_fft.hpp"
int main() {
    int n; std::cin >> n;
    std::vector<long long> a(n+1), b(n+1);
    for (int i = 1; i <= n; ++i) std::cin >> a[i] >> b[i];
    const auto c = cp::convolution_fft(std::vector<double>(a.begin(), a.end()), std::vector<double>(b.begin(), b.end()));
    for (int k = 1; k <= 2*n; ++k) std::cout << std::llround(c[k]) << '\n';
}
