#pragma once
// 説明: docs/combinatorics/binomial_prefix_sum.md
#include <algorithm>
#include <stdexcept>
#include <vector>
#include "binomial.hpp"
namespace cp {
// 0<=n<=max_n、sum_{j=0}^k C(n,j)。Pascal表の各行prefix、合成数法可。
class BinomialPrefixSum {
    long long modulus_;std::vector<std::vector<long long>> prefix_;
public:
    explicit BinomialPrefixSum(int max_n,long long modulus=998244353):modulus_(modulus){PascalBinomial table(max_n,modulus);prefix_=table.rows();for(auto& row:prefix_)for(std::size_t k=1;k<row.size();++k)row[k]=combinatorics_detail::add_mod(row[k],row[k-1],modulus);}
    int max_n()const{return static_cast<int>(prefix_.size())-1;}
    long long operator()(long long n,long long k)const{if(n<0 || k<0)return 0;if(n>max_n())throw std::out_of_range("binomial prefix outside table");
    return prefix_[static_cast<std::size_t>(n)][static_cast<std::size_t>(std::min(n,k))];}
    long long sum(long long n,long long l,long long r)const{if(n<0 || l>=r || r<=0)return 0;if(n>max_n())throw std::out_of_range("binomial prefix outside table");
    l=std::max(0LL,l);r=std::min(n+1,r);if(l>=r)return 0;return combinatorics_detail::subtract_mod((*this)(n,r-1),(*this)(n,l-1),modulus_);}
};
} // namespace cp
