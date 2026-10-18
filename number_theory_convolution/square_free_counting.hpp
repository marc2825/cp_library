#pragma once
// 説明: docs/number_theory_convolution/square_free_counting.md
#include <cstdint>
#include <limits>
#include <stdexcept>
#include "../basic_utility/integer_kth_root.hpp"
#include "../number_theory_basic/mobius.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
// 1..nの平方因子を持たない整数の個数。O(sqrt n)のmu表と和。
inline std::uint64_t square_free_count(std::uint64_t n,std::uint64_t max_root=5000000){auto root=integer_kth_root(n,2);if(root>max_root || root>static_cast<std::uint64_t>(std::numeric_limits<int>::max()))throw std::length_error("square-free root budget exceeded");
    auto mu=mobius_sieve(static_cast<int>(root));OptimizationWide result=0;for(std::uint64_t i=1;i<=root;++i)result=optimization_detail::add(result,OptimizationWide(mu[i])*OptimizationWide(n/(i*i)));return optimization_detail::narrow<std::uint64_t>(result);}
} // namespace cp
