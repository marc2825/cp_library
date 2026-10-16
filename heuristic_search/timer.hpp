#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace cp {
// Wall time in seconds. Clock injection supports deterministic tests.
template<class Clock = std::chrono::steady_clock> class BasicTimer {
    static_assert(Clock::is_steady, "Timer needs a monotonic clock");
    typename Clock::time_point start_;
    double limit_;
    static void validate(double seconds) {
        if (std::isnan(seconds) || seconds < 0) throw std::invalid_argument("invalid timer limit");
    }
public:
    explicit BasicTimer(double limit_seconds = std::numeric_limits<double>::infinity())
        : start_(Clock::now()), limit_(limit_seconds) { validate(limit_); }
    void reset() { start_ = Clock::now(); }
    void reset(double seconds) { validate(seconds); limit_ = seconds; reset(); }
    double elapsed() const { return std::chrono::duration<double>(Clock::now() - start_).count(); }
    double limit() const { return limit_; }
    double remaining() const { return std::max(0.0, limit_ - elapsed()); }
    bool expired() const { return elapsed() >= limit_; }
    double progress() const {
        if (limit_ == 0) return 1;
        if (std::isinf(limit_)) return 0;
        return std::clamp(elapsed() / limit_, 0.0, 1.0);
    }
};
using Timer = BasicTimer<>;
} // namespace cp
