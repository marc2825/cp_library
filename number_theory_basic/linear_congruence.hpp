#pragma once
// 説明: docs/number_theory_basic/linear_congruence.md
#include <numeric>
#include <optional>
#include <stdexcept>
#include "mod_inverse.hpp"
#include "mod_pow.hpp"
namespace cp {
struct LinearCongruenceResult {long long first,step,count;};
// ax=b mod mの全解: first+t*step (0<=t<count)、[0,m)内。
inline std::optional<LinearCongruenceResult> linear_congruence(long long a,long long b,long long modulus){
    if(modulus<=0)throw std::invalid_argument("congruence modulus must be positive");
    a=detail::normalized_remainder(a,modulus);b=detail::normalized_remainder(b,modulus);long long g=std::gcd(a,modulus);if(b%g)return std::nullopt;long long step=modulus/g,first=step==1?0:mod_multiply(b/g,*mod_inverse(a/g,step),step);return LinearCongruenceResult{first,step,g};
}
} // namespace cp
