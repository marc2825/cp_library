#pragma once
// 説明: docs/basic_utility/fraction_binary_search.md
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include "stern_brocot_tree.hpp"

namespace cp {
// 0/1をtrue、1/0をfalseとする単調境界。分子・分母ともlimit以下。
// 有限の正の候補だけpredicate(p,q)へ渡す。返値は最大true・最小false。
template<class Predicate>
std::pair<NonnegativeFraction,NonnegativeFraction> fraction_binary_search(std::uint64_t limit, Predicate predicate) {
    if (!limit) throw std::invalid_argument("positive fraction limit required");
    NonnegativeFraction l{0,1}, r{1,0};
    while (l.numerator<=limit-r.numerator && l.denominator<=limit-r.denominator) {
        bool side=predicate(l.numerator+r.numerator,l.denominator+r.denominator);
        auto a=side?l:r, b=side?r:l;
        std::uint64_t cap=limit;
        if (b.numerator) cap=std::min(cap,(limit-a.numerator)/b.numerator);
        if (b.denominator) cap=std::min(cap,(limit-a.denominator)/b.denominator);
        auto same=[&](std::uint64_t k) { return bool(predicate(a.numerator+k*b.numerator,a.denominator+k*b.denominator))==side; };
        std::uint64_t lo=1, hi=1;
        while (hi<cap && same(hi)) { lo=hi; hi=hi>cap/2?cap:2*hi; }
        if (same(hi)) lo=hi;
        else while (hi-lo>1) { auto m=lo+(hi-lo)/2; if (same(m)) lo=m; else hi=m; }
        a.numerator+=lo*b.numerator; a.denominator+=lo*b.denominator;
        (side?l:r)=a;
    }
    return {l,r};
}
} // namespace cp
