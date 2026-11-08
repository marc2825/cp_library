#pragma once
// 説明: docs/basic_utility/stern_brocot_tree.md
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
struct NonnegativeFraction { std::uint64_t numerator, denominator; }; // 1/0は+∞。
// L/Rの連続区間。root=1/1。既約でない入力も正規化する。
inline std::vector<std::pair<char,std::uint64_t>> stern_brocot_path(std::uint64_t p, std::uint64_t q) {
    if (!p || !q) throw std::invalid_argument("positive fraction required");
    auto g = std::gcd(p,q); p /= g; q /= g;
    std::vector<std::pair<char,std::uint64_t>> result;
    while (p != q) {
        if (p < q) { auto k = (q-1)/p; result.emplace_back('L',k); q -= k*p; }
        else { auto k = (p-1)/q; result.emplace_back('R',k); p -= k*q; }
    }
    return result;
}
inline NonnegativeFraction stern_brocot_fraction(const std::vector<std::pair<char,std::uint64_t>>& path) {
    std::uint64_t a=0,b=1,c=1,d=0,limit=std::numeric_limits<std::uint64_t>::max();
    for (auto [direction,k] : path) {
        if (direction!='L' && direction!='R') throw std::invalid_argument("expected L or R");
        auto add = [=](std::uint64_t x,std::uint64_t y) {
            if (y && k>(limit-x)/y) throw std::overflow_error("Stern-Brocot bound");
            return x+k*y;
        };
        if (direction=='L') { c=add(c,a); d=add(d,b); }
        else { a=add(a,c); b=add(b,d); }
    }
    if (a>limit-c || b>limit-d) throw std::overflow_error("Stern-Brocot fraction");
    return {a+c,b+d};
}
} // namespace cp
