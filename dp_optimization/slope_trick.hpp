#pragma once
#include <functional>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
// 整数傾きの凸区分線形関数。初期値f=0。単位hingeを二つのheapで管理。
class SlopeTrick {
    std::priority_queue<long long> left_;
    std::priority_queue<long long,std::vector<long long>,std::greater<long long>> right_;
    long long shift_left_=0,shift_right_=0,minimum_=0;
    long long left() const { return detail::dp_add(left_.top(),shift_left_); }
    long long right() const { return detail::dp_add(right_.top(),shift_right_); }
    void push_left(long long x) { left_.push(detail::dp_subtract(x,shift_left_)); }
    void push_right(long long x) { right_.push(detail::dp_subtract(x,shift_right_)); }
public:
    long long minimum() const { return minimum_; }
    // nulloptは左端/右端の非有界を表す。
    std::pair<std::optional<long long>,std::optional<long long>> argmin() const {
        return {left_.empty()?std::optional<long long>{}:left(),right_.empty()?std::optional<long long>{}:right()};
    }
    void add_constant(long long c) { minimum_=detail::dp_add(minimum_,c); }
    void add_a_minus_x(long long a) {
        if (!right_.empty() && right()<a) {
            const auto r=right(); add_constant(detail::dp_subtract(a,r));
            right_.pop(); push_right(a); push_left(r);
        } else push_left(a);
    }
    void add_x_minus_a(long long a) {
        if (!left_.empty() && left()>a) {
            const auto l=left(); add_constant(detail::dp_subtract(l,a));
            left_.pop(); push_left(a); push_right(l);
        } else push_right(a);
    }
    void add_abs(long long a) { add_a_minus_x(a); add_x_minus_a(a); }
    void prefix_min() { decltype(right_) empty; right_.swap(empty); }
    void suffix_min() { decltype(left_) empty; left_.swap(empty); }
    // f(x) <- min_{x-b<=y<=x-a}f(y)、a<=b。a=bなら平行移動。
    void shift(long long a,long long b) {
        if (a>b) throw std::invalid_argument("inverted slope shift interval");
        auto l=detail::dp_add(shift_left_,a),r=detail::dp_add(shift_right_,b);
        shift_left_=l; shift_right_=r;
    }
    void shift(long long a) { shift(a,a); }
    // 評価と非破壊mergeはheapのコピーを含む。通常更新はO(log N)。
    long long evaluate(long long x) const {
        auto l=left_; auto r=right_; auto value=minimum_;
        while (!l.empty()) { auto p=detail::dp_add(l.top(),shift_left_); if(p>x) value=detail::dp_add(value,detail::dp_subtract(p,x)); l.pop(); }
        while (!r.empty()) { auto p=detail::dp_add(r.top(),shift_right_); if(p<x) value=detail::dp_add(value,detail::dp_subtract(x,p)); r.pop(); }
        return value;
    }
    void merge(const SlopeTrick& other) {
        auto l=other.left_; auto r=other.right_; const auto a=other.shift_left_,b=other.shift_right_,c=other.minimum_;
        add_constant(c);
        while (!l.empty()) { add_a_minus_x(detail::dp_add(l.top(),a)); l.pop(); }
        while (!r.empty()) { add_x_minus_a(detail::dp_add(r.top(),b)); r.pop(); }
    }
};
} // namespace cp
