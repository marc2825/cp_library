#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
namespace cp::numerical {
enum class Status {
    converged,
    invalid_bracket,
    zero_derivative,
    stagnation,
    max_iterations,
    max_evaluations,
    max_depth,
    non_finite,
    singular
};
inline const char *status_name(Status s) {
    switch (s) {
    case Status::converged:
        return "converged";
    case Status::invalid_bracket:
        return "invalid_bracket";
    case Status::zero_derivative:
        return "zero_derivative";
    case Status::stagnation:
        return "stagnation";
    case Status::max_iterations:
        return "max_iterations";
    case Status::max_evaluations:
        return "max_evaluations";
    case Status::max_depth:
        return "max_depth";
    case Status::non_finite:
        return "non_finite";
    case Status::singular:
        return "singular";
    }
    return "unknown";
}
struct ErrorTolerance {
    long double absolute = 1e-12L, relative = 1e-12L;
    ErrorTolerance(long double a = 1e-12L, long double r = 1e-12L) : absolute(a), relative(r) { validate(); }
    void validate() const {
        if (!std::isfinite(absolute) || !std::isfinite(relative) || absolute < 0 || relative < 0)
            throw std::invalid_argument("tolerances must be finite and nonnegative");
    }
    long double threshold(long double scale) const {
        validate();
        return absolute + relative * std::abs(scale);
    }
};
// NaN is never equal; identical infinities are equal. No overflowing a-b or tolerance*scale.
inline bool almost_equal(long double a, long double b, ErrorTolerance t = {}) {
    t.validate();
    if (a == b)
        return true;
    if (!std::isfinite(a) || !std::isfinite(b))
        return false;
    const auto scale = std::max(std::abs(a), std::abs(b));
    if (scale == 0)
        return true;
    const auto diff = std::abs(a - b);
    if (std::isfinite(diff))
        return diff <= t.absolute || diff / scale <= t.relative + t.absolute / scale;
    return std::abs(a / scale - b / scale) <= t.relative + t.absolute / scale;
}
namespace detail {
inline void input(long double x) {
    if (!std::isfinite(x))
        throw std::invalid_argument("finite input required");
}
struct Stop {
    Status status;
};
inline long double checked(long double x) {
    if (!std::isfinite(x))
        throw Stop{Status::non_finite};
    return x;
}
inline long double midpoint(long double a, long double b) {
    return std::signbit(a) == std::signbit(b) ? a + (b - a) / 2 : (a + b) / 2;
}
inline bool opposite(long double a, long double b) { return std::signbit(a) != std::signbit(b); }
inline long double half_distance(long double a, long double b) { return std::abs(a / 2 - b / 2); }
} // namespace detail
} // namespace cp::numerical
