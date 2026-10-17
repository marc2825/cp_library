#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"
#include "../number_theory_basic/mod_inverse.hpp"
#include "../number_theory_basic/mod_pow.hpp"

namespace cp {
// sumは固定点総数mod modulus。群の位数が可逆であること。modulus=1も可。
inline long long burnside_average(long long sum,long long group_order,long long modulus=998244353) {
    if (group_order<=0 || modulus<=0) throw std::invalid_argument("invalid group order or modulus");
    auto inverse=mod_inverse(group_order,modulus);
    if (!inverse) throw std::domain_error("group order must be invertible modulo modulus");
    return mod_multiply(sum,*inverse,modulus);
}
// 群要素を0..group_order-1で列挙。fixed(i)は非負の固定点数、またはその剰余。
template<class Fixed> long long burnside_orbits(int group_order,Fixed fixed,long long modulus=998244353) {
    if (group_order<=0 || modulus<=0) throw std::invalid_argument("invalid group order or modulus");
    long long sum=0;
    for (int i=0;i<group_order;++i) {
        long long count=fixed(i);
        if (count<0) throw std::invalid_argument("negative fixed-point count");
        sum=combinatorics_detail::add_mod(sum,count%modulus,modulus);
    }
    return burnside_average(sum,group_order,modulus);
}
// 正確な整数平均。総和がuint64内に収まり、群としての固定点数であること。
inline std::uint64_t burnside_orbits_exact(const std::vector<std::uint64_t>& fixed_counts) {
    if (fixed_counts.empty()) throw std::invalid_argument("empty group");
    std::uint64_t sum=0;
    for (auto x:fixed_counts) {
        if (sum>std::numeric_limits<std::uint64_t>::max()-x) throw std::overflow_error("Burnside fixed-point sum overflow");
        sum+=x;
    }
    if (sum%fixed_counts.size()) throw std::domain_error("nonintegral Burnside average");
    return sum/fixed_counts.size();
}
} // namespace cp
