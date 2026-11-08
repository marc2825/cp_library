#pragma once
#include <algorithm>
#include <iterator>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include "../basic_utility/big_integer.hpp"

namespace cp {
// 重複breakpointを重みで圧縮するSlope Trick。座標・傾き・値は厳密BigInt。
// 単位hingeを重み回反復しない。更新は交差して消費するbreakpoint数に比例。
class WeightedSlopeTrick {
    std::map<BigInt, BigInt> left_, right_;
    BigInt shift_left_, shift_right_, minimum_;
public:
    const BigInt& minimum() const { return minimum_; }
    std::size_t breakpoint_count() const { return left_.size() + right_.size(); }
    std::pair<std::optional<BigInt>, std::optional<BigInt>> argmin() const {
        std::optional<BigInt> l, r;
        if (!left_.empty()) l = left_.rbegin()->first + shift_left_;
        if (!right_.empty()) r = right_.begin()->first + shift_right_;
        return {l,r};
    }
    void add_constant(const BigInt& c) { minimum_ += c; }
    void add_a_minus_x(BigInt a, BigInt weight = 1) {
        if (weight < 0) throw std::invalid_argument("negative slope weight");
        while (weight > 0 && !right_.empty()) {
            auto it = right_.begin(); const auto x = it->first + shift_right_;
            if (x >= a) break;
            const auto moved = std::min(weight, it->second);
            minimum_ += (a-x)*moved; weight -= moved;
            it->second -= moved; if (it->second == 0) right_.erase(it);
            right_[a-shift_right_] += moved;
            left_[x-shift_left_] += moved;
        }
        if (weight > 0) left_[a-shift_left_] += weight;
    }
    void add_x_minus_a(BigInt a, BigInt weight = 1) {
        if (weight < 0) throw std::invalid_argument("negative slope weight");
        while (weight > 0 && !left_.empty()) {
            auto it = std::prev(left_.end()); const auto x = it->first + shift_left_;
            if (x <= a) break;
            const auto moved = std::min(weight, it->second);
            minimum_ += (x-a)*moved; weight -= moved;
            it->second -= moved; if (it->second == 0) left_.erase(it);
            left_[a-shift_left_] += moved;
            right_[x-shift_right_] += moved;
        }
        if (weight > 0) right_[a-shift_right_] += weight;
    }
    void add_abs(BigInt a, BigInt weight = 1) { add_a_minus_x(a, weight); add_x_minus_a(a, weight); }
    void prefix_min() { right_.clear(); }
    void suffix_min() { left_.clear(); }
    // f(x) <- min_{x-b<=y<=x-a} f(y)
    void shift(const BigInt& a, const BigInt& b) {
        if (a > b) throw std::invalid_argument("inverted slope shift interval");
        shift_left_ += a; shift_right_ += b;
    }
    void shift(const BigInt& a) { shift(a,a); }
    BigInt evaluate(const BigInt& x) const {
        auto value = minimum_;
        for (const auto& p : left_) { auto a = p.first + shift_left_; if (a > x) value += (a-x)*p.second; }
        for (const auto& p : right_) { auto a = p.first + shift_right_; if (a < x) value += (x-a)*p.second; }
        return value;
    }
    void merge(const WeightedSlopeTrick& other) {
        const auto copy = other; // 自分自身とのmergeも可。
        add_constant(copy.minimum_);
        for (const auto& p : copy.left_) add_a_minus_x(p.first + copy.shift_left_, p.second);
        for (const auto& p : copy.right_) add_x_minus_a(p.first + copy.shift_right_, p.second);
    }
    // f(x) <- f(-x)。整数極値でもBigIntなので符号反転可能。
    void reflect() {
        std::map<BigInt, BigInt> l, r;
        for (const auto& p : right_) l.emplace(-p.first, p.second);
        for (const auto& p : left_) r.emplace(-p.first, p.second);
        left_.swap(l); right_.swap(r);
        auto offset = shift_left_; shift_left_ = -shift_right_; shift_right_ = -offset;
    }
};
} // namespace cp
