#include <iostream>
#include <vector>
#include "dp_optimization/divide_conquer_dp.hpp"

int main() {
    std::vector<long long> prefix{0,1,3,6};
    auto result=cp::divide_conquer_dp(3,2,[&](int l,int r) {
        auto sum=prefix[r]-prefix[l]; return sum*sum;
    });
    std::cout << *result.dp[2][3];
    auto parts=result.partitions(2,3);
    for (auto [l,r]:*parts) std::cout << " " << l << ":" << r;
    std::cout << "\n";
}
