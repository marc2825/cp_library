#include <iostream>
#include "dp_optimization/online_offline_dp.hpp"
int main() {
    auto dp=cp::online_offline_convolution_dp(std::vector<int>{1,0,0,0,0},std::vector<int>{0,1,1,1,1});
    std::cout << dp.back() << '\n';
}
