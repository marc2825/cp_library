#pragma once
#include "stable_arithmetic.hpp"
#include <optional>
#include <queue>
#include <utility>
#include <vector>
namespace cp::numerical {
struct IntegrationResult {
    Status status = Status::converged;
    long double value = 0;
    std::optional<long double> error_estimate;
    std::size_t evaluations = 0, panels = 0;
};
struct IntegrationOptions {
    ErrorTolerance tolerance;
    std::size_t max_evaluations = 100000, max_depth = 50;
};
namespace detail {
inline long double integration_product(long double a, long double b, long double weight) {
    int ea = 0, eb = 0;
    const auto ma = std::frexp(a, &ea), mb = std::frexp(b, &eb);
    return checked(std::ldexp(ma * mb * weight, ea + eb));
}
inline void integration_add(CompensatedSum &sum, long double x) {
    try {
        sum.add(x);
    } catch (const std::overflow_error &) {
        throw Stop{Status::non_finite};
    }
}
} // namespace detail
// Composite Simpson with an even positive number of equal-width subintervals.
template <class F> IntegrationResult simpson(F f, long double a, long double b, std::size_t n) {
    detail::input(a);
    detail::input(b);
    if (!n || n % 2)
        throw std::invalid_argument("Simpson needs a positive even subdivision count");
    IntegrationResult r;
    if (a == b)
        return r;
    try {
        const auto h = detail::checked((b - a) / n);
        if (!h)
            throw detail::Stop{Status::stagnation};
        CompensatedSum sum;
        auto previous = a;
        for (std::size_t i = 0; i <= n; ++i) {
            const auto x = i == n ? b : std::fma(h, static_cast<long double>(i), a);
            if (i && (a < b ? x <= previous : x >= previous))
                throw detail::Stop{Status::stagnation};
            previous = x;
            ++r.evaluations;
            const auto y = detail::checked(f(x));
            // Scale each term before summing, avoiding an unnecessary n-fold growth.
            detail::integration_add(sum, detail::integration_product(y, h,
                                                                     (i == 0 || i == n ? 1.L
                                                                      : i % 2          ? 4.L
                                                                                       : 2.L) /
                                                                         3));
        }
        r.value = sum.value();
        r.panels = n / 2;
    } catch (detail::Stop s) {
        r.status = s.status;
    }
    return r;
}
// Global error budget, refinements in descending estimated error. Estimates are not rigorous bounds.
template <class F>
IntegrationResult adaptive_simpson(F f, long double a, long double b, IntegrationOptions o = {}) {
    detail::input(a);
    detail::input(b);
    o.tolerance.validate();
    IntegrationResult out;
    out.error_estimate = 0;
    if (a == b)
        return out;
    const int sign = a < b ? 1 : -1;
    if (a > b)
        std::swap(a, b);
    struct Panel {
        long double a, b, fa, fm, fb, fl, fr, value, error;
        std::size_t depth;
        bool operator<(const Panel &p) const { return error < p.error; }
    };
    std::priority_queue<Panel> queue;
    CompensatedSum total, errors;
    bool roundoff = false;
    auto evaluate = [&](long double x) {
        if (out.evaluations == o.max_evaluations)
            throw detail::Stop{Status::max_evaluations};
        ++out.evaluations;
        return detail::checked(f(x));
    };
    auto make = [&](long double l, long double r, long double fl, long double fm, long double fr,
                    std::size_t depth) {
        const auto m = detail::midpoint(l, r), lm = detail::midpoint(l, m), rm = detail::midpoint(m, r);
        if (lm == l || lm == m || rm == m || rm == r)
            throw detail::Stop{Status::stagnation};
        const auto ql = evaluate(lm), qr = evaluate(rm), h = detail::checked(r - l);
        CompensatedSum coarse_sum, fine_sum;
        for (const auto &term : {std::pair{fl, 1.L / 6}, std::pair{fm, 2.L / 3}, std::pair{fr, 1.L / 6}})
            detail::integration_add(coarse_sum, detail::integration_product(term.first, h, term.second));
        for (const auto &term : {std::pair{fl, 1.L / 12}, std::pair{ql, 1.L / 3}, std::pair{fm, 1.L / 6},
                                 std::pair{qr, 1.L / 3}, std::pair{fr, 1.L / 12}})
            detail::integration_add(fine_sum, detail::integration_product(term.first, h, term.second));
        const auto coarse = coarse_sum.value(), fine = fine_sum.value();
        const auto delta = detail::checked((fine - coarse) / 15);
        return Panel{l, r, fl, fm, fr, ql, qr, detail::checked(fine + delta), std::abs(delta), depth};
    };
    auto push = [&](const Panel &p) {
        if (p.depth < o.max_depth)
            queue.push(p);
    };
    try {
        const auto m = detail::midpoint(a, b);
        const auto fa = evaluate(a), fm = evaluate(m), fb = evaluate(b);
        const auto first = make(a, b, fa, fm, fb, 0);
        detail::integration_add(total, first.value);
        detail::integration_add(errors, first.error);
        out.panels = 1;
        push(first);
        for (;;) {
            out.value = sign * total.value();
            out.error_estimate = std::max(0.L, errors.value());
            if (*out.error_estimate <= o.tolerance.threshold(out.value))
                return out;
            if (queue.empty()) {
                out.status = roundoff ? Status::stagnation : Status::max_depth;
                return out;
            }
            const auto p = queue.top();
            queue.pop();
            if (o.max_evaluations - out.evaluations < 4) {
                out.status = Status::max_evaluations;
                return out;
            }
            const auto mid = detail::midpoint(p.a, p.b);
            // Child quarter points must remain distinct before any evaluations or state changes.
            const auto lmid = detail::midpoint(p.a, mid), rmid = detail::midpoint(mid, p.b);
            if (!(detail::midpoint(p.a, lmid) > p.a && detail::midpoint(p.a, lmid) < lmid &&
                  detail::midpoint(lmid, mid) > lmid && detail::midpoint(lmid, mid) < mid &&
                  detail::midpoint(mid, rmid) > mid && detail::midpoint(mid, rmid) < rmid &&
                  detail::midpoint(rmid, p.b) > rmid && detail::midpoint(rmid, p.b) < p.b)) {
                roundoff = true;
                continue;
            }
            const auto left = make(p.a, mid, p.fa, p.fl, p.fm, p.depth + 1);
            const auto right = make(mid, p.b, p.fm, p.fr, p.fb, p.depth + 1);
            detail::integration_add(total, -p.value);
            detail::integration_add(total, left.value);
            detail::integration_add(total, right.value);
            detail::integration_add(errors, -p.error);
            detail::integration_add(errors, left.error);
            detail::integration_add(errors, right.error);
            ++out.panels;
            push(left);
            push(right);
        }
    } catch (detail::Stop s) {
        out.status = s.status;
    }
    if (!out.panels)
        out.error_estimate = std::numeric_limits<long double>::infinity();
    return out;
}
} // namespace cp::numerical
