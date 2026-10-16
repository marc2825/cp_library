#define PROBLEM "https://atcoder.jp/contests/atc001/tasks/fft_c"
#include <cmath>
#include <complex>
#include <iostream>
#include <vector>
#include "convolution/convolution_ll.hpp"
int main() {
    int n; std::cin >> n;
    std::vector<long long> a(n+1), b(n+1);
    for (int i = 1; i <= n; ++i) std::cin >> a[i] >> b[i];
    const auto c = cp::convolution_ll(a, b);
    for (int k = 1; k <= 2*n; ++k) std::cout << c[k] << '\n';
}
