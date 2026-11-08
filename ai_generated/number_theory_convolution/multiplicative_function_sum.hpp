#pragma once
// 説明: docs/number_theory_convolution/multiplicative_function_sum.md
#include <cstdint>
#include <vector>
#include "../number_theory_basic/smallest_prime_factor.hpp"
#include "detail/arithmetic.hpp"
namespace cp {
template<class T> struct MultiplicativeFunctionSums{std::vector<T> values,prefix;};
// prime_power(p,e,p^e)から乗法的関数fと全prefixを構築。f(1)=1。
template<class T,class PrimePower> MultiplicativeFunctionSums<T> multiplicative_function_sums(int limit,PrimePower prime_power){
    SmallestPrimeFactor spf(limit);MultiplicativeFunctionSums<T> result{std::vector<T>(static_cast<std::size_t>(limit)+1,T(0)),std::vector<T>(static_cast<std::size_t>(limit)+1,T(0))};std::vector<int> exponent(static_cast<std::size_t>(limit)+1);std::vector<std::uint64_t> power(static_cast<std::size_t>(limit)+1,1);if(limit)result.values[1]=T(1);
    for(int i=2;i<=limit;++i){int p=spf[i],q=i/p;if(q%p==0){exponent[i]=exponent[q]+1;power[i]=power[q]*static_cast<unsigned>(p);}else{exponent[i]=1;power[i]=static_cast<unsigned>(p);}result.values[i]=detail::arithmetic_multiply(result.values[static_cast<std::size_t>(i)/power[i]],T(prime_power(p,exponent[i],power[i])));}
    for(int i=1;i<=limit;++i){result.prefix[i]=result.prefix[i-1];detail::arithmetic_add_assign(result.prefix[i],result.values[i]);}return result;
}
} // namespace cp
