#define PROBLEM "https://atcoder.jp/contests/dp/tasks/dp_z"
#include <iostream>
#include <vector>
#include "dp_optimization/li_chao_tree.hpp"
int main(){int n;long long c;std::cin>>n>>c;std::vector<long long> h(n),dp(n);for(auto& x:h)std::cin>>x;cp::LiChaoTree<> lines(h);for(int i=0;i<n;++i){if(i)dp[i]=*lines.query(h[i])+h[i]*h[i]+c;lines.add_line(-2*h[i],dp[i]+h[i]*h[i]);}std::cout<<dp.back()<<"\n";}
