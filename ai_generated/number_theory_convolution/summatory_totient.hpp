#pragma once
// 説明: docs/number_theory_convolution/summatory_totient.md
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include "../number_theory_basic/euler_phi.hpp"
#include "../basic_utility/quotient_enumeration.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
// Du Jiao: sum phi(d)*floor(n/d)=n(n+1)/2。複数queryでmemoを共有。
class SummatoryTotient {
    std::vector<OptimizationWide> prefix_;std::unordered_map<std::uint64_t,OptimizationWide> memo_;
public:
    explicit SummatoryTotient(int precompute=1000000){if(precompute<1)throw std::invalid_argument("positive totient precomputation required");
    auto phi=euler_phi_sieve(precompute);prefix_.resize(phi.size());for(std::size_t i=1;i<phi.size();++i)prefix_[i]=prefix_[i-1]+phi[i];}
    OptimizationWide operator()(std::uint64_t n){if(n<prefix_.size())return prefix_[static_cast<std::size_t>(n)];auto it=memo_.find(n);if(it!=memo_.end())return it->second;using W=OptimizationWide;W result=n%2?W(n)*((W(n)+1)/2):(W(n)/2)*(W(n)+1);for_each_quotient(n,[&](std::uint64_t l,std::uint64_t r,std::uint64_t q){if(l==1)l=2;if(l<=r)result=optimization_detail::sub(result,optimization_detail::mul(W(r-l)+1,(*this)(q)));});memo_[n]=result;return result;}
    std::size_t memo_size()const{return memo_.size();}
};
} // namespace cp
