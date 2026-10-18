#pragma once
// 説明: docs/dp_optimization/online_offline_dp.md
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../convolution/convolution.hpp"
namespace cp {
// finish(i)の昇順確定と、確定済み[l,m)から未確定[m,r)への一括transfer。
template<class Finish,class Transfer> void cdq_online_offline(int n,Finish finish,Transfer transfer){
    if(n<0)throw std::invalid_argument("negative CDQ size");
    auto solve=[&](auto&& self,int l,int r)->void{if(r-l==1){finish(l);return;}if(l==r)return;int m=l+(r-l)/2;self(self,l,m);transfer(l,m,r);self(self,m,r);};solve(solve,0,n);
}
// dp[i]=initial[i]+sum_{j<i}dp[j]*kernel[i-j]。kernel[0]は使わない。
template<int Mod=998244353,int Root=3> std::vector<int> online_offline_convolution_dp(const std::vector<int>& initial,const std::vector<int>& kernel){
    if(initial.size()>static_cast<std::size_t>(2147483647))throw std::length_error("CDQ input too large");
    std::vector<int> dp(initial.size());for(std::size_t i=0;i<initial.size();++i)dp[i]=convolution_detail::normalize<Mod>(initial[i]);
    cdq_online_offline(static_cast<int>(dp.size()),[](int){},[&](int l,int m,int r){std::vector<int> a(dp.begin()+l,dp.begin()+m),b(static_cast<std::size_t>(r-l));for(int i=1;i<r-l && i<static_cast<int>(kernel.size());++i)b[i]=kernel[i];auto c=convolution<Mod,Root>(a,b);for(int i=m;i<r;++i)dp[i]=static_cast<int>((static_cast<long long>(dp[i])+c[i-l])%Mod);});return dp;
}
} // namespace cp
