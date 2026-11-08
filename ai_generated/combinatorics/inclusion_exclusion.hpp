#pragma once
#include <cstdint>
#include <stdexcept>
#include "detail/arithmetic.hpp"

namespace cp {
// intersection(mask)=選んだ集合の共通部分の大きさ。maskを昇順に訪問する。
// union は非空maskのみ、complement はmask=0を全体集合として含める。
template <class T, class Intersection>
T inclusion_exclusion_union(int sets, Intersection intersection) {
    if (sets < 0 || sets >= 64) throw std::invalid_argument("number of sets must be in [0,63]");
    T result(0);
    const auto end = std::uint64_t(1) << sets;
    for (std::uint64_t mask = 1; mask < end; ++mask) {
        const T value = intersection(mask);
        if (combinatorics_detail::odd_popcount(mask)) combinatorics_detail::accumulate<false>(result, value);
        else combinatorics_detail::accumulate<true>(result, value);
    }
    return result;
}
template <class T, class Intersection>
T inclusion_exclusion_complement(int sets, Intersection intersection) {
    if (sets < 0 || sets >= 64) throw std::invalid_argument("number of sets must be in [0,63]");
    T result(0);
    const auto end = std::uint64_t(1) << sets;
    for (std::uint64_t mask = 0; mask < end; ++mask) {
        const T value = intersection(mask);
        if (combinatorics_detail::odd_popcount(mask)) combinatorics_detail::accumulate<true>(result, value);
        else combinatorics_detail::accumulate<false>(result, value);
    }
    return result;
}
} // namespace cp
