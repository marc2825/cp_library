#pragma once
#include "../basic_utility/binary_rational.hpp"
#include "../basic_utility/detail/rational_polynomial.hpp"
#include "common.hpp"
#include <utility>
#include <vector>
namespace cp::numerical {
namespace polynomial_detail {
using namespace rational_polynomial_detail;
inline std::vector<Polynomial> sturm(Polynomial p) {
    std::vector<Polynomial> chain{p, derivative(p)};
    while (!chain.back().empty()) {
        auto r = divide(chain[chain.size() - 2], chain.back()).second;
        for (auto &x : r)
            x = -x;
        normalize(r, true);
        chain.push_back(std::move(r));
    }
    chain.pop_back();
    return chain;
}
inline std::size_t variations(const std::vector<Polynomial> &chain, const BigRational &x) {
    int previous = 0;
    std::size_t changes = 0;
    for (const auto &p : chain) {
        const int sign = evaluate(p, x).sign();
        if (sign) {
            changes += previous && previous != sign;
            previous = sign;
        }
    }
    return changes;
}
} // namespace polynomial_detail
struct RealRootInterval {
    BigRational lower, upper; // (lower,upper], or an exact singleton lower==upper
    std::size_t count = 0;    // distinct roots; >1 only in unfinished clusters
    long double value = 0;    // midpoint approximation; the rational interval is the certificate
};
struct PolynomialRootOptions {
    ErrorTolerance tolerance;
    std::size_t max_subdivisions = 100000, max_degree = 64;
};
struct RealRootsResult {
    Status status = Status::converged;
    bool all_real = false;
    std::size_t distinct_roots = 0, subdivisions = 0;
    std::vector<RealRootInterval> intervals;
};
// Exact Sturm counts for rational coefficients; no tolerance-based merging of nearby roots.
inline RealRootsResult isolate_real_roots(std::vector<BigRational> coefficients,
                                          PolynomialRootOptions o = {}) {
    using namespace polynomial_detail;
    o.tolerance.validate();
    trim(coefficients);
    RealRootsResult out;
    if (coefficients.empty()) {
        out.all_real = true;
        return out;
    }
    if (coefficients.size() == 1)
        return out;
    if (coefficients.size() - 1 > o.max_degree)
        throw std::length_error("polynomial degree budget exceeded");
    auto p = divide(coefficients, gcd(coefficients, derivative(coefficients))).first;
    normalize(p);
    const auto chain = sturm(p);
    BigRational bound = 1;
    for (std::size_t i = 0; i + 1 < p.size(); ++i)
        bound = std::max(bound, BigRational(1) + abs(p[i]));
    bound += 1; // strict Cauchy bound, neither endpoint is a root
    const auto atol = rational_from_binary(o.tolerance.absolute),
               rtol = rational_from_binary(o.tolerance.relative);
    struct Work {
        BigRational a, b;
        std::size_t va, vb;
    };
    std::vector<Work> stack{{-bound, bound, variations(chain, -bound), variations(chain, bound)}};
    out.distinct_roots = stack[0].va - stack[0].vb;
    auto emit = [&](BigRational a, BigRational b, std::size_t count) {
        if (!count)
            return;
        const auto mid = (a + b) / 2;
        long double value;
        try {
            value = rational_to_long_double(mid);
        } catch (const std::overflow_error &) {
            value = std::copysign(std::numeric_limits<long double>::infinity(), mid.sign());
            out.status = Status::non_finite;
        }
        out.intervals.push_back({std::move(a), std::move(b), count, value});
    };
    while (!stack.empty()) {
        auto w = std::move(stack.back());
        stack.pop_back();
        const auto count = w.va - w.vb;
        if (!count)
            continue;
        if (count == 1 && !evaluate(p, w.b)) {
            emit(w.b, w.b, 1);
            continue;
        }
        if (count == 1 && w.b - w.a <= atol + rtol * std::max(abs(w.a), abs(w.b))) {
            emit(w.a, w.b, 1);
            continue;
        }
        if (out.subdivisions == o.max_subdivisions) {
            if (out.status == Status::converged)
                out.status = Status::max_iterations;
            emit(w.a, w.b, count);
            for (const auto &pending : stack)
                emit(pending.a, pending.b, pending.va - pending.vb);
            stack.clear();
            break;
        }
        const auto mid = (w.a + w.b) / 2;
        const auto vm = variations(chain, mid);
        ++out.subdivisions;
        if (vm > w.vb)
            stack.push_back({mid, w.b, vm, w.vb});
        if (w.va > vm)
            stack.push_back({w.a, mid, w.va, vm});
    }
    std::sort(out.intervals.begin(), out.intervals.end(), [](const auto &a, const auto &b) {
        return a.lower != b.lower ? a.lower < b.lower : a.upper < b.upper;
    });
    return out;
}
// Coefficients are ascending powers. Their represented binary values are converted exactly.
inline RealRootsResult real_polynomial_roots(const std::vector<long double> &coefficients,
                                             PolynomialRootOptions o = {}) {
    std::vector<BigRational> exact;
    for (auto x : coefficients)
        exact.push_back(rational_from_binary(x));
    return isolate_real_roots(std::move(exact), o);
}
} // namespace cp::numerical
