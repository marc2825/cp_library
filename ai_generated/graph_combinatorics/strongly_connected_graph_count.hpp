#pragma once
// 説明: docs/graph_combinatorics/strongly_connected_graph_count.md
#include <vector>
#include "../basic_utility/big_integer.hpp"
#include "detail/labeled_count.hpp"
namespace cp {
// ループなし単純有向グラフ、逆向き2辺は独立に許す。s[0]=0,s[1]=1。
template<class T=BigInt> std::vector<T> strongly_connected_graph_counts(int n) {
    auto choose=graph_combinatorics_detail::count_binomial<T>(n);
    std::vector<T> eta(n+1,T(0)),answer(n+1,T(0)); T total(1),power(1);
    for(int i=1;i<=n;++i) {
        total=total*power*power; eta[i]=total; T factor=power;
        for(int k=i-1;k>=1;--k) { eta[i]-=choose[i][k]*factor*eta[k]; factor=factor*power; }
        answer[i]=eta[i];
        for(int k=1;k<i;++k) answer[i]+=choose[i-1][k-1]*answer[k]*eta[i-k];
        power=power+power;
    }
    return answer;
}
} // namespace cp
