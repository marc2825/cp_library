#pragma once
#include "../numerical_analysis/common.hpp"
#include <optional>
#include <vector>
namespace cp {
struct FractionalPoint {
    long double numerator = 0, denominator = 1;
    std::vector<long double> solution;
};
enum class FractionalStatus {
    converged,
    infeasible,
    invalid_bracket,
    iteration_limit,
    stagnation,
    non_finite
};
struct FractionalOptions {
    numerical::ErrorTolerance tolerance;
    long double denominator_lower_bound = 1e-12L;
    std::size_t max_iterations = 128;
};
struct FractionalResult {
    FractionalStatus status = FractionalStatus::iteration_limit;
    bool has_solution = false;
    FractionalPoint point;
    long double ratio = 0, lower = -std::numeric_limits<long double>::infinity(),
                upper = std::numeric_limits<long double>::infinity();
    std::size_t evaluations = 0;
};
namespace fractional_detail {
inline void validate(const FractionalOptions &o) {
    o.tolerance.validate();
    if (!std::isfinite(o.denominator_lower_bound) || o.denominator_lower_bound <= 0)
        throw std::invalid_argument("positive denominator lower bound required");
}
inline long double check(const FractionalPoint &p, long double parameter, const FractionalOptions &o) {
    if (!std::isfinite(p.numerator) || !std::isfinite(p.denominator))
        throw numerical::detail::Stop{numerical::Status::non_finite};
    if (p.denominator < o.denominator_lower_bound)
        throw std::invalid_argument("oracle violated denominator lower bound");
    return numerical::detail::checked(std::fma(-parameter, p.denominator, p.numerator));
}
inline void incumbent(FractionalResult &r, const FractionalPoint &p) {
    const auto ratio = numerical::detail::checked(p.numerator / p.denominator);
    if (!r.has_solution || ratio > r.ratio) {
        r.has_solution = true;
        r.point = p;
        r.ratio = ratio;
        r.lower = std::max(r.lower, ratio);
    }
}
} // namespace fractional_detail
// Oracle(lambda) returns a global maximizer of numerator-lambda*denominator, or nullopt for empty domain.
// All feasible denominators must be >= denominator_lower_bound; optima must be attained.
template <class Oracle>
FractionalResult fractional_dinkelbach(Oracle oracle, long double start = 0, FractionalOptions o = {}) {
    fractional_detail::validate(o);
    numerical::detail::input(start);
    FractionalResult out;
    try {
        auto lambda = start;
        for (std::size_t i = 0; i < o.max_iterations; i++) {
            const std::optional<FractionalPoint> point = oracle(lambda);
            ++out.evaluations;
            if (!point) {
                out.status = FractionalStatus::infeasible;
                return out;
            }
            const auto residual = fractional_detail::check(*point, lambda, o);
            fractional_detail::incumbent(out, *point);
            const auto bound = residual <= 0 ? lambda : lambda + residual / o.denominator_lower_bound;
            out.upper = std::min(out.upper, bound);
            // Roundoff may move bounds past one another; never claim a negative gap as convergence.
            if (out.upper < out.lower) {
                out.status = FractionalStatus::stagnation;
                return out;
            }
            if (residual == 0 || numerical::almost_equal(out.lower, out.upper, o.tolerance)) {
                out.status = FractionalStatus::converged;
                return out;
            }
            const auto next = point->numerator / point->denominator;
            if (next == lambda) {
                out.status = FractionalStatus::stagnation;
                return out;
            }
            lambda = next;
        }
    } catch (numerical::detail::Stop) {
        out.status = FractionalStatus::non_finite;
    }
    return out;
}
template <class Oracle>
FractionalResult fractional_bisection(Oracle oracle, long double lower, long double upper,
                                      FractionalOptions o = {}) {
    fractional_detail::validate(o);
    numerical::detail::input(lower);
    numerical::detail::input(upper);
    if (lower > upper)
        throw std::invalid_argument("reversed ratio bounds");
    FractionalResult out;
    out.lower = lower;
    out.upper = upper;
    auto evaluate = [&](long double value) {
        const std::optional<FractionalPoint> p = oracle(value);
        ++out.evaluations;
        return p;
    };
    try {
        for (int endpoint = 0; endpoint < 2; endpoint++) {
            const auto lambda = endpoint ? upper : lower;
            const auto point = evaluate(lambda);
            if (!point) {
                out.status = FractionalStatus::infeasible;
                return out;
            }
            const auto residual = fractional_detail::check(*point, lambda, o);
            fractional_detail::incumbent(out, *point);
            if (residual == 0) {
                out.lower = out.upper = out.ratio;
                out.status = FractionalStatus::converged;
                return out;
            }
            if ((!endpoint && residual < 0) || (endpoint && residual > 0)) {
                out.status = FractionalStatus::invalid_bracket;
                return out;
            }
        }
        for (std::size_t i = 0;; i++) {
            if (out.upper < out.lower) {
                out.status = FractionalStatus::stagnation;
                return out;
            }
            if (numerical::almost_equal(out.lower, out.upper, o.tolerance)) {
                out.status = FractionalStatus::converged;
                return out;
            }
            if (i == o.max_iterations)
                return out;
            const auto mid = numerical::detail::midpoint(out.lower, out.upper);
            if (mid == out.lower || mid == out.upper) {
                out.status = FractionalStatus::stagnation;
                return out;
            }
            const auto point = evaluate(mid);
            if (!point) {
                out.status = FractionalStatus::infeasible;
                return out;
            }
            const auto residual = fractional_detail::check(*point, mid, o);
            fractional_detail::incumbent(out, *point);
            if (residual >= 0)
                out.lower = std::max(out.lower, mid);
            else
                out.upper = mid;
            if (residual == 0) {
                out.lower = out.upper = out.ratio;
                out.status = FractionalStatus::converged;
                return out;
            }
        }
    } catch (numerical::detail::Stop) {
        out.status = FractionalStatus::non_finite;
        return out;
    }
}
} // namespace cp
