#pragma once
#include "incremental_score.hpp"
#include <vector>
namespace cp {
// Midpoint quantization of a uniform variate. Probability error <= 1/(2*2^bits)
// in real arithmetic; floating rounding of logarithms/comparisons is additional.
class FastAnnealingAcceptance {
    unsigned bits_;
    std::vector<long double> thresholds_;

  public:
    explicit FastAnnealingAcceptance(unsigned bits = 12) : bits_(bits) {
        if (bits < 1 || bits > 24)
            throw std::invalid_argument("acceptance table bits must be 1..24");
        const std::size_t n = std::size_t(1) << bits;
        thresholds_.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
            thresholds_.push_back(-std::log((static_cast<long double>(i) + .5L) / n));
    }
    unsigned bits() const { return bits_; }
    long double probability_error_bound() const { return .5L / thresholds_.size(); }
    const std::vector<long double> &thresholds() const { return thresholds_; }
    bool accept_index(long double loss, long double temperature, std::size_t index) const {
        if (!std::isfinite(loss) || loss < 0 || !std::isfinite(temperature) || temperature <= 0)
            throw std::invalid_argument("invalid annealing loss/temperature");
        const auto threshold = thresholds_.at(index);
        return loss == 0 || loss < temperature * threshold;
    }
    bool operator()(long double loss, long double temperature, Random &random) const {
        if (!std::isfinite(loss) || loss < 0 || !std::isfinite(temperature) || temperature <= 0)
            throw std::invalid_argument("invalid annealing loss/temperature");
        if (loss == 0)
            return true;
        const auto index = random() >> (64 - bits_);
        return loss < temperature * thresholds_[index];
    }
};
} // namespace cp
