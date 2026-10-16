#include <algorithm>
#include <iostream>
#include <vector>
#include "set_function/sos_dp.hpp"
int main() {
    std::vector<int> sum{3, 1, 4, 2}, best = sum;
    cp::sos_dp(sum);
    cp::sos_dp(best, [](int a, int b) { return std::max(a, b); });
    for (int x : sum) std::cout << x << ' ';
    std::cout << '\n';
    for (int x : best) std::cout << x << ' ';
    std::cout << '\n';
}
