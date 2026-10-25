#pragma once
#include <stdexcept>
#include <vector>
namespace cp { namespace graph_combinatorics_detail {
template<class T> std::vector<std::vector<T>> count_binomial(int n) {
    if(n<0) throw std::invalid_argument("negative graph count bound");
    std::vector<std::vector<T>> c(n+1);
    for(int i=0;i<=n;++i) { c[i].assign(i+1,T(1)); for(int j=1;j<i;++j) c[i][j]=c[i-1][j-1]+c[i-1][j]; }
    return c;
}
} } // namespace cp::graph_combinatorics_detail
