#pragma once
#include <cassert>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <vector>
#include "../range_query/fenwick_tree.hpp"

namespace cp {
// 0..n-1の順列の辞書順ランク。uint64に全順位が入るn<=20。
inline std::vector<std::uint64_t> permutation_factorials(int n) {
    if (n<0 || n>20) throw std::invalid_argument("permutation size must be 0..20");
    std::vector<std::uint64_t> f(n+1,1);
    for (int i=1;i<=n;++i) f[i]=f[i-1]*i;
    return f;
}
inline std::uint64_t permutation_rank(const std::vector<int>& p) {
    int n=int(p.size()); auto f=permutation_factorials(n);
    FenwickTree<int> bit(n);
    for (int i=0;i<n;++i) bit.add(i,1);
    std::uint64_t result=0;
    for (int i=0;i<n;++i) {
        if (p[i]<0 || p[i]>=n || bit.sum(p[i],p[i]+1)!=1)
            throw std::invalid_argument("not a permutation");
        result+=std::uint64_t(bit.sum(0,p[i]))*f[n-1-i]; bit.add(p[i],-1);
    }
    return result;
}
inline std::vector<int> permutation_unrank(int n, std::uint64_t rank) {
    auto f=permutation_factorials(n);
    if (rank>=f[n]) throw std::out_of_range("permutation rank");
    FenwickTree<int> bit(n); for (int i=0;i<n;++i) bit.add(i,1);
    std::vector<int> p;
    for (int i=n-1;i>=0;--i) {
        int k=int(rank/f[i]); rank%=f[i];
        int x=bit.lower_bound(k+1); p.push_back(x); bit.add(x,-1);
    }
    return p;
}
} // namespace cp
