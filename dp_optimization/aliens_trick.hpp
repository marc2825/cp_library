#pragma once
#include <cstdint>
#include <optional>
#include <stdexcept>
#include "detail/arithmetic.hpp"

namespace cp {
struct PenalizedDPResult { long long value; int count; };
struct AliensTrickResult { long long value,penalty; PenalizedDPResult relaxed; };
// min_k(F(k)+penalty*k)のoracle。同値なら個数が多い解を返す。
// Fが離散凸、整数penaltyで支持される場合のexact-k。範囲外はnullopt。
template<class Oracle> std::optional<AliensTrickResult> aliens_trick(
        int k,long long low,long long high,Oracle oracle) {
    if (k<0 || low>high) throw std::invalid_argument("invalid WQS target or bounds");
    auto first=oracle(low),last=oracle(high);
    if (first.count<0 || last.count<0) throw std::invalid_argument("negative oracle count");
    if (first.count<k || (last.count>k)) return std::nullopt;
    // last>=kなら境界上のexact-k。そうでなければ最後のcount>=kを探す。
    if (last.count==k) return AliensTrickResult{detail::dp_subtract(last.value,detail::dp_multiply(high,k)),high,last};
    while (low<high) {
        auto distance=static_cast<std::uint64_t>(high)-static_cast<std::uint64_t>(low);
        auto half=static_cast<long long>(distance/2);
        long long middle=detail::dp_add(detail::dp_add(low,half),static_cast<long long>(distance%2));
        auto result=oracle(middle);
        if (result.count<0) throw std::invalid_argument("negative oracle count");
        if (result.count>=k) low=middle; else high=middle-1;
    }
    auto result=oracle(low);
    return AliensTrickResult{detail::dp_subtract(result.value,detail::dp_multiply(low,k)),low,result};
}
} // namespace cp
