#pragma once
#include "common.hpp"
#include <utility>
namespace cp::numerical {
enum class RootCriterion { none, exact, residual, interval };
struct RootOptions {
    ErrorTolerance x_tolerance;
    long double value_tolerance = 0;
    std::size_t max_iterations = 128, max_evaluations = 512;
    void validate() const {
        x_tolerance.validate();
        detail::input(value_tolerance);
        if (value_tolerance < 0)
            throw std::invalid_argument("negative residual tolerance");
    }
};
struct RootResult {
    Status status = Status::max_iterations;
    RootCriterion criterion = RootCriterion::none;
    long double root = 0, value = std::numeric_limits<long double>::quiet_NaN(), lower = 0, upper = 0;
    std::size_t iterations = 0, evaluations = 0;
    bool bracketed = false;
};
namespace detail {
inline bool root_value(RootResult &r, const RootOptions &o) {
    if (r.value == 0 || std::abs(r.value) <= o.value_tolerance) {
        r.status = Status::converged;
        r.criterion = r.value == 0 ? RootCriterion::exact : RootCriterion::residual;
        return true;
    }
    return false;
}
template <class F> long double evaluate(F &f, long double x, RootResult &r, const RootOptions &o) {
    if (r.evaluations == o.max_evaluations)
        throw Stop{Status::max_evaluations};
    ++r.evaluations;
    return checked(f(x));
}
template <bool Brent, class F> RootResult bracket_root(F f, long double a, long double b, RootOptions o) {
    input(a);
    input(b);
    o.validate();
    if (a > b)
        throw std::invalid_argument("reversed root interval");
    RootResult r;
    r.lower = a;
    r.upper = b;
    r.root = a;
    try {
        auto fa = evaluate(f, a, r, o);
        r.value = fa;
        if (root_value(r, o))
            return r;
        auto fb = evaluate(f, b, r, o);
        r.root = b;
        r.value = fb;
        if (root_value(r, o))
            return r;
        if (!opposite(fa, fb)) {
            r.status = Status::invalid_bracket;
            return r;
        }
        r.bracketed = true;
        if (std::abs(fa) < std::abs(fb)) {
            std::swap(a, b);
            std::swap(fa, fb);
        }
        long double c = a, fc = fa, d = c;
        bool bisected = true;
        for (;;) {
            r.root = b;
            r.value = fb;
            r.lower = std::min(a, b);
            r.upper = std::max(a, b);
            if (root_value(r, o))
                return r;
            if (almost_equal(a, b, o.x_tolerance)) {
                r.status = Status::converged;
                r.criterion = RootCriterion::interval;
                return r;
            }
            if (r.iterations == o.max_iterations)
                return r;
            auto s = midpoint(a, b);
            bool use_bisection = true;
            if constexpr (Brent) {
                const auto scale = std::max({std::abs(fa), std::abs(fb), std::abs(fc)});
                const auto x = fa / scale, y = fb / scale, z = fc / scale;
                long double candidate;
                if (x != z && y != z && x != y)
                    candidate = a * (y / (x - y)) * (z / (x - z)) + b * (x / (y - x)) * (z / (y - z)) +
                                c * (x / (z - x)) * (y / (z - y));
                else {
                    const auto t = -x / (y - x);
                    candidate = (1 - t) * a + t * b;
                }
                const auto guard = a * 0.75L + b * 0.25L;
                const auto previous = bisected ? half_distance(b, c) : half_distance(c, d);
                if (std::isfinite(candidate) && candidate > std::min(guard, b) &&
                    candidate < std::max(guard, b) && half_distance(candidate, b) < previous / 2 &&
                    !almost_equal(bisected ? b : c, bisected ? c : d, o.x_tolerance)) {
                    s = candidate;
                    use_bisection = false;
                }
            }
            if (s == a || s == b) {
                r.status = Status::stagnation;
                return r;
            }
            const auto fs = evaluate(f, s, r, o);
            ++r.iterations;
            d = c;
            c = b;
            fc = fb;
            bisected = use_bisection;
            if (opposite(fa, fs)) {
                b = s;
                fb = fs;
            } else {
                a = s;
                fa = fs;
            }
            if (std::abs(fa) < std::abs(fb)) {
                std::swap(a, b);
                std::swap(fa, fb);
            }
        }
    } catch (Stop s) {
        r.status = s.status;
        return r;
    }
}
} // namespace detail
template <class F> RootResult bisection(F f, long double lower, long double upper, RootOptions o = {}) {
    return detail::bracket_root<false>(std::move(f), lower, upper, o);
}
template <class F> RootResult brent(F f, long double lower, long double upper, RootOptions o = {}) {
    return detail::bracket_root<true>(std::move(f), lower, upper, o);
}
// Newton has no certified bracket. A zero derivative or rounding-stalled step is explicit failure.
template <class F, class D> RootResult newton(F f, D derivative, long double start, RootOptions o = {}) {
    detail::input(start);
    o.validate();
    RootResult r;
    r.root = start;
    r.lower = r.upper = start;
    try {
        r.value = detail::evaluate(f, r.root, r, o);
        for (;;) {
            if (detail::root_value(r, o))
                return r;
            if (r.iterations == o.max_iterations)
                return r;
            const auto df = detail::evaluate(derivative, r.root, r, o);
            if (!df) {
                r.status = Status::zero_derivative;
                return r;
            }
            const auto next = detail::checked(r.root - r.value / df);
            if (next == r.root) {
                r.status = Status::stagnation;
                return r;
            }
            const auto value = detail::evaluate(f, next, r, o);
            ++r.iterations;
            r.root = next;
            r.lower = r.upper = next;
            r.value = value;
        }
    } catch (detail::Stop s) {
        r.status = s.status;
        return r;
    }
}
} // namespace cp::numerical
