#pragma once
// 説明: docs/graph_combinatorics/connected_graph_count.md
#include <vector>
#include "../basic_utility/big_integer.hpp"
#include "detail/labeled_count.hpp"
namespace cp {
// ラベル付き単純無向連結グラフの数 c[0..n]。c[0]=0。除算不要の環演算。
template<class T=BigInt> std::vector<T> connected_graph_counts(int n) {
    auto choose=graph_combinatorics_detail::count_binomial<T>(n);
    std::vector<T> total(n+1,T(1)),answer(n+1,T(0)); T power(1);
    for(int i=1;i<=n;++i) {
        total[i]=total[i-1]*power; power=power+power; answer[i]=total[i];
        for(int k=1;k<i;++k) answer[i]-=choose[i-1][k-1]*answer[k]*total[i-k];
    }
    return answer;
}
} // namespace cp
