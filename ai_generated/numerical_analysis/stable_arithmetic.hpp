#pragma once
#include "common.hpp"
#include <vector>
namespace cp::numerical {
class KahanSum {
    long double sum_ = 0, correction_ = 0;

  public:
    void add(long double x) {
        detail::input(x);
        const auto y = x - correction_, next = sum_ + y, correction = (next - sum_) - y;
        if (!std::isfinite(next) || !std::isfinite(correction))
            throw std::overflow_error("Kahan sum overflow");
        sum_ = next;
        correction_ = correction;
    }
    long double value() const { return sum_; }
    void clear() { sum_ = correction_ = 0; }
};
// Neumaier's compensated sum also handles a new term larger than the running sum.
class CompensatedSum {
    long double sum_ = 0, correction_ = 0;

  public:
    void add(long double x) {
        detail::input(x);
        const auto next = sum_ + x;
        const auto error = std::abs(sum_) >= std::abs(x) ? (sum_ - next) + x : (x - next) + sum_;
        const auto correction = correction_ + error;
        if (!std::isfinite(next) || !std::isfinite(correction) || !std::isfinite(next + correction))
            throw std::overflow_error("compensated sum overflow");
        sum_ = next;
        correction_ = correction;
    }
    long double value() const { return sum_ + correction_; }
    void clear() { sum_ = correction_ = 0; }
};
template <class Iterator> long double compensated_sum(Iterator first, Iterator last) {
    CompensatedSum s;
    for (; first != last; ++first)
        s.add(*first);
    return s.value();
}
inline long double log_sum_exp(const std::vector<long double> &values) {
    const auto inf = std::numeric_limits<long double>::infinity();
    long double largest = -inf;
    for (auto x : values) {
        if (std::isnan(x))
            throw std::invalid_argument("NaN log weight");
        largest = std::max(largest, x);
    }
    if (!std::isfinite(largest))
        return largest; // empty/all -inf => -inf, any +inf => +inf
    CompensatedSum s;
    bool skipped = false;
    for (auto x : values) {
        if (!skipped && x == largest) {
            skipped = true;
            continue;
        }
        s.add(std::exp(x - largest));
    }
    const auto answer = largest + std::log1p(s.value());
    if (!std::isfinite(answer))
        throw std::overflow_error("log-sum-exp overflow");
    return answer;
}
} // namespace cp::numerical
