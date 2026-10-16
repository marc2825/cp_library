#pragma once
#include <algorithm>
#include <vector>
#include "../number_theory_convolution/detail/arithmetic.hpp"

namespace cp {
namespace min_plus_detail {
template<class T> T sum(T a,const T& b) { detail::arithmetic_add_assign(a,b); return a; }
}
// c[k]=min_{i+j=k}(a[i]+b[j])。両列が離散凸ならO(n+m)。
template<class T> std::vector<T> min_plus_convolution_convex_convex(const std::vector<T>& a,const std::vector<T>& b) {
    if(a.empty() || b.empty()) return {};
    std::vector<T> c; c.reserve(a.size()+b.size()-1); std::size_t i=0,j=0; c.push_back(min_plus_detail::sum(a[0],b[0]));
    while(i+1<a.size() || j+1<b.size()) {
        if(j+1==b.size() || (i+1<a.size() && min_plus_detail::sum(a[i+1],b[j])<min_plus_detail::sum(a[i],b[j+1]))) ++i; else ++j;
        c.push_back(min_plus_detail::sum(a[i],b[j]));
    }
    return c;
}
// aは任意、bは離散凸。帯状Monge行列の単調最小位置を直接探索、O((n+m)log(n+m))。
template<class T> std::vector<T> min_plus_convolution_arbitrary_convex(const std::vector<T>& a,const std::vector<T>& b) {
    if(a.empty() || b.empty()) return {};
    int n=int(a.size()),m=int(b.size()); std::vector<T> c(n+m-1,T(0));
    auto solve=[&](auto&& self,int top,int bottom,int left,int right)->void {
        if(top>=bottom) return;
        int k=top+(bottom-top)/2,lo=std::max(left,k-m+1),hi=std::min(right,k+1),best=lo;
        T value=min_plus_detail::sum(a[lo],b[k-lo]);
        for(int i=lo+1;i<hi;++i) { T v=min_plus_detail::sum(a[i],b[k-i]); if(v<value) { value=v; best=i; } }
        c[k]=value; self(self,top,k,left,best+1); self(self,k+1,bottom,best,right);
    };
    solve(solve,0,n+m-1,0,n); return c;
}
template<class T> std::vector<T> min_plus_convolution_convex_arbitrary(const std::vector<T>& a,const std::vector<T>& b) { return min_plus_convolution_arbitrary_convex(b,a); }
} // namespace cp
