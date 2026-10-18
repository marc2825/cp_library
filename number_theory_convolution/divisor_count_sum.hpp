#pragma once
// 説明: docs/number_theory_convolution/divisor_count_sum.md
#include <cstdint>
#include "../basic_utility/quotient_enumeration.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
// sum_{k=1}^n tau(k)=sum_{i=1}^n floor(n/i)。128bitで返す。
inline OptimizationWide divisor_count_sum(std::uint64_t n){OptimizationWide result=0;for_each_quotient(n,[&](std::uint64_t l,std::uint64_t r,std::uint64_t q){result=optimization_detail::add(result,optimization_detail::mul(OptimizationWide(r-l)+1,OptimizationWide(q)));});return result;}
} // namespace cp
