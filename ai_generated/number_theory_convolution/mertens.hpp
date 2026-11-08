#pragma once
// 説明: docs/number_theory_convolution/mertens.md
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include "../number_theory_basic/mobius.hpp"
#include "../basic_utility/quotient_enumeration.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
// M(n)=1-sum_{i=2}^n M(floor(n/i))。0に対して0。
class MertensFunction {
    std::vector<long long> prefix_;std::unordered_map<std::uint64_t,OptimizationWide> memo_;
public:
    explicit MertensFunction(int precompute=1000000){if(precompute<1)throw std::invalid_argument("positive Mertens precomputation required");
    auto mu=mobius_sieve(precompute);prefix_.resize(mu.size());for(std::size_t i=1;i<mu.size();++i)prefix_[i]=prefix_[i-1]+mu[i];}
    OptimizationWide operator()(std::uint64_t n){if(n<prefix_.size())return prefix_[static_cast<std::size_t>(n)];auto it=memo_.find(n);if(it!=memo_.end())return it->second;OptimizationWide result=1;for_each_quotient(n,[&](std::uint64_t l,std::uint64_t r,std::uint64_t q){if(l==1)l=2;if(l<=r)result=optimization_detail::sub(result,optimization_detail::mul(OptimizationWide(r-l)+1,(*this)(q)));});memo_[n]=result;return result;}
    std::size_t memo_size()const{return memo_.size();}
};
} // namespace cp
