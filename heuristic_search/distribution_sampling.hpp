#pragma once
#include "random.hpp"
#include <cmath>
#include <queue>
#include <vector>
namespace cp {
// Integer-weight Vose table: exact discrete probabilities, no floating threshold rounding.
// Sum must fit uint64_t. Products during construction use unsigned __int128.
class AliasSampler {
    std::vector<std::uint64_t> threshold_;
    std::vector<std::size_t> alias_;
    std::uint64_t total_ = 0;

  public:
    explicit AliasSampler(const std::vector<std::uint64_t> &weights) {
        if (weights.empty())
            throw std::invalid_argument("empty alias distribution");
        for (auto w : weights) {
            if (w > UINT64_MAX - total_)
                throw std::overflow_error("alias weight total");
            total_ += w;
        }
        if (!total_)
            throw std::invalid_argument("zero alias weight total");
        using Wide = __uint128_t;
        const auto n = weights.size();
        std::vector<Wide> mass;
        std::vector<std::size_t> small, large;
        threshold_.resize(n);
        alias_.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            mass.push_back(Wide(weights[i]) * n);
            (mass.back() < total_ ? small : large).push_back(i);
        }
        while (!small.empty() && !large.empty()) {
            const auto s = small.back(), l = large.back();
            small.pop_back();
            large.pop_back();
            threshold_[s] = static_cast<std::uint64_t>(mass[s]);
            alias_[s] = l;
            mass[l] -= total_ - mass[s];
            (mass[l] < total_ ? small : large).push_back(l);
        }
        for (auto i : large) {
            threshold_[i] = total_;
            alias_[i] = i;
        }
        for (auto i : small) {
            threshold_[i] = total_;
            alias_[i] = i;
        }
    }
    std::size_t size() const { return threshold_.size(); }
    std::uint64_t total_weight() const { return total_; }
    const std::vector<std::uint64_t> &thresholds() const { return threshold_; }
    const std::vector<std::size_t> &aliases() const { return alias_; }
    std::size_t choose(std::size_t column, std::uint64_t coin) const {
        if (column >= size() || coin >= total_)
            throw std::out_of_range("alias column/coin");
        return coin < threshold_[column] ? column : alias_[column];
    }
    std::size_t sample(Random &random) const {
        const auto column = random.uniform(size());
        return choose(column, random.uniform(total_));
    }
};
namespace sampling_detail {
// Midpoints of 2^52 bins: exactly representable in double and strictly inside (0,1).
inline double open_uniform(Random &random) { return (static_cast<double>(random() >> 12) + .5) * 0x1.0p-52; }
inline void positive(double x) {
    if (!std::isfinite(x) || x <= 0)
        throw std::invalid_argument("positive finite sampling parameter required");
}
inline double finite(double x) {
    if (!std::isfinite(x))
        throw std::overflow_error("nonfinite sample");
    return x;
}
} // namespace sampling_detail
// Successive weighted draws without replacement, via independent exponential clocks.
// Zero weights are excluded; output is ordered by clock time (not sorted by ID).
inline std::vector<std::size_t> weighted_sample_without_replacement(const std::vector<double> &weights,
                                                                    std::size_t count, Random &random) {
    std::size_t positive = 0;
    for (auto w : weights) {
        if (!std::isfinite(w) || w < 0)
            throw std::invalid_argument("invalid sample weight");
        positive += w > 0;
    }
    if (count > positive)
        throw std::invalid_argument("sample exceeds positive support");
    if (!count)
        return {};
    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry> heap;
    for (std::size_t i = 0; i < weights.size(); ++i)
        if (weights[i] > 0) {
            const double key =
                std::log(-std::log(sampling_detail::open_uniform(random))) - std::log(weights[i]);
            Entry e{key, i};
            if (heap.size() < count)
                heap.push(e);
            else if (e < heap.top()) {
                heap.pop();
                heap.push(e);
            }
        }
    std::vector<std::size_t> out;
    while (!heap.empty()) {
        out.push_back(heap.top().second);
        heap.pop();
    }
    std::reverse(out.begin(), out.end());
    return out;
}
inline double sample_exponential(double rate, Random &random) {
    sampling_detail::positive(rate);
    return sampling_detail::finite(-std::log(sampling_detail::open_uniform(random)) / rate);
}
// Exponential(rate) conditioned on lower <= X <= upper; use memorylessness and expm1/log1p.
inline double sample_truncated_exponential(double rate, double lower, double upper, Random &random) {
    sampling_detail::positive(rate);
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower < 0 || upper < lower)
        throw std::invalid_argument("invalid truncated exponential interval");
    if (lower == upper)
        return lower;
    const double width = upper - lower, z = rate * width, u = sampling_detail::open_uniform(random);
    // As z tends to zero the conditional distribution tends to uniform.
    // For tiny z, expand the normalized inverse CDF. This also avoids subnormal
    // products losing all precision before division by a tiny rate.
    const double offset =
        z < 1e-8 ? width * (u - z * u * (1 - u) / 2) : -std::log1p(u * std::expm1(-z)) / rate;
    return std::clamp(sampling_detail::finite(lower + offset), lower, upper);
}
class NormalSampler {
    bool cached_ = false;
    double spare_ = 0;

  public:
    void reset() { cached_ = false; }
    double sample(Random &random, double mean = 0, double standard_deviation = 1) {
        if (!std::isfinite(mean) || !std::isfinite(standard_deviation) || standard_deviation < 0)
            throw std::invalid_argument("invalid normal parameters");
        if (standard_deviation == 0)
            return mean;
        double z;
        if (cached_) {
            z = spare_;
            cached_ = false;
        } else {
            const auto radius = std::sqrt(-2 * std::log(sampling_detail::open_uniform(random))),
                       angle = 2 * std::acos(-1.0) * sampling_detail::open_uniform(random);
            z = radius * std::cos(angle);
            spare_ = radius * std::sin(angle);
            cached_ = true;
        }
        return sampling_detail::finite(std::fma(standard_deviation, z, mean));
    }
};
} // namespace cp
