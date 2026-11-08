#include <iostream>
#include <vector>
#include "dp_optimization/interval_dp.hpp"
int main() {
    std::vector<long long> a{10,20,30}, prefix(4);
    for (int i=0;i<3;++i) prefix[i+1]=prefix[i]+a[i];
    auto result = cp::interval_dp(3, [&](int l,int,int r) { return prefix[r]-prefix[l]; });
    std::cout << result.value() << '\n';
    for (auto [l,k,r] : result.merge_order()) std::cout << l << ' ' << k << ' ' << r << '\n';
}
