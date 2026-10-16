#define PROBLEM "https://atcoder.jp/contests/abc179/tasks/abc179_c"
#include <iostream>
#include <numeric>
#include <vector>
#include "number_theory_convolution/dirichlet_convolution.hpp"
int main() {
    int n; std::cin >> n;
    std::vector<long long> one(n, 1); one[0] = 0;
    const auto count = cp::dirichlet_convolution(one, one);
    std::cout << std::accumulate(count.begin(), count.end(), 0LL) << '\n';
}
