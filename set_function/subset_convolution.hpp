#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "detail/transform.hpp"
#include "subset_zeta_transform.hpp"
#include "subset_mobius_transform.hpp"

namespace cp {
// h[S]=sum_{T subset S} a[T]*b[S\T]。長さは同じ正の2冪。O(k^2 2^k)。
// 係数は環。標準整数の途中の超過は例外。docs/set_function/subset_convolution.md
template<class T> std::vector<T> subset_convolution(const std::vector<T>& a,const std::vector<T>& b) {
    set_function_detail::validate_size(a.size()); if(a.size()!=b.size()) throw std::invalid_argument("subset convolution size mismatch");
    std::size_t count=a.size(); int k=0; for(auto m=count;m>1;m>>=1) ++k;
    std::vector<int> rank(count); for(std::size_t s=1;s<count;++s) rank[s]=rank[s>>1]+int(s&1);
    std::vector<std::vector<T>> x(k+1,std::vector<T>(count,T(0))),y=x;
    for(std::size_t s=0;s<count;++s) { x[rank[s]][s]=a[s]; y[rank[s]][s]=b[s]; }
    for(int i=0;i<=k;++i) { subset_zeta_transform(x[i]); subset_zeta_transform(y[i]); }
    std::vector<T> result(count,T(0));
    for(int degree=0;degree<=k;++degree) {
        std::vector<T> h(count,T(0));
        for(int i=0;i<=degree;++i) for(std::size_t s=0;s<count;++s) detail::arithmetic_add_assign(h[s],detail::arithmetic_multiply(x[i][s],y[degree-i][s]));
        subset_mobius_transform(h);
        for(std::size_t s=0;s<count;++s) if(rank[s]==degree) result[s]=h[s];
    }
    return result;
}
} // namespace cp
