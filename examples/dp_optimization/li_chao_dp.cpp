#include <iostream>
#include <vector>
#include "dp_optimization/li_chao_tree.hpp"

int main() {
    std::vector<long long> h{10,30,20},dp(h.size());
    cp::LiChaoTree<> lines(h);
    for (int i=0;i<static_cast<int>(h.size());++i) {
        if (i) dp[i]=*lines.query(h[i])+h[i]*h[i]+10;
        lines.add_line(-2*h[i],dp[i]+h[i]*h[i]);
    }
    std::cout << dp.back() << "\n";
}
