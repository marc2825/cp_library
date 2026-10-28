#pragma once
#include "rational/kernel.hpp"
#include <array>
#include <limits>

namespace cp::geometry::robust {
// Exact value of a finite radix-2 long double, including subnormals. No decimal conversion.
inline BigRational dyadic(long double x) {
    floating::input(x);
    static_assert(std::numeric_limits<long double>::radix == 2, "binary floating required");
    if (!x)
        return 0;
    int exponent;
    long double mantissa = std::frexp(std::abs(x), &exponent);
    BigInt::backend_type n = 0, d = 1;
    const int digits = std::numeric_limits<long double>::digits;
    for (int i = 0; i < digits; ++i) {
        mantissa *= 2;
        n <<= 1;
        if (mantissa >= 1) {
            ++n;
            mantissa -= 1;
        }
    }
    exponent -= digits;
    if (exponent >= 0)
        n <<= exponent;
    else
        d <<= -exponent;
    if (x < 0)
        n = -n;
    return {BigInt(std::move(n)), BigInt(std::move(d))};
}
inline rational::Point exact_value(floating::Point p) { return {dyadic(p.x), dyadic(p.y)}; }
namespace detail {
struct Interval {
    long double lo, hi;
    Interval(long double x) : lo(x), hi(x) {}
    Interval(long double l, long double h) : lo(l), hi(h) {}
    static Interval entire() {
        const auto inf = std::numeric_limits<long double>::infinity();
        return {-inf, inf};
    }
    static Interval widen(long double l, long double h) {
        if (std::isnan(l) || std::isnan(h))
            return entire();
        const auto inf = std::numeric_limits<long double>::infinity();
        return {std::nextafter(l, -inf), std::nextafter(h, inf)};
    }
    friend Interval operator+(Interval a, Interval b) { return widen(a.lo + b.lo, a.hi + b.hi); }
    friend Interval operator-(Interval a, Interval b) { return widen(a.lo - b.hi, a.hi - b.lo); }
    friend Interval operator*(Interval a, Interval b) {
        std::array<long double, 4> v{a.lo * b.lo, a.lo * b.hi, a.hi * b.lo, a.hi * b.hi};
        for (auto x : v)
            if (std::isnan(x))
                return entire();
        return widen(*std::min_element(v.begin(), v.end()), *std::max_element(v.begin(), v.end()));
    }
    int certain_sign() const { return lo > 0 ? 1 : hi < 0 ? -1 : 0; }
};
template <class T> T orient(T ax, T ay, T bx, T by, T cx, T cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}
template <class T> T circle(T ax, T ay, T bx, T by, T cx, T cy, T dx, T dy) {
    ax = ax - dx;
    ay = ay - dy;
    bx = bx - dx;
    by = by - dy;
    cx = cx - dx;
    cy = cy - dy;
    return (ax * ax + ay * ay) * (bx * cy - by * cx) - (bx * bx + by * by) * (ax * cy - ay * cx) +
           (cx * cx + cy * cy) * (ax * by - ay * bx);
}
} // namespace detail
inline int orientation(exact::Point a, exact::Point b, exact::Point c) {
    return detail::orient(BigInt(a.x), BigInt(a.y), BigInt(b.x), BigInt(b.y), BigInt(c.x), BigInt(c.y))
        .sign();
}
inline int incircle(exact::Point a, exact::Point b, exact::Point c, exact::Point d) {
    return detail::circle(BigInt(a.x), BigInt(a.y), BigInt(b.x), BigInt(b.y), BigInt(c.x), BigInt(c.y),
                          BigInt(d.x), BigInt(d.y))
        .sign();
}
inline int orientation(floating::Point a, floating::Point b, floating::Point c) {
    for (auto p : {a, b, c})
        floating::validate(p);
    using I = detail::Interval;
    const int sign = detail::orient(I(a.x), I(a.y), I(b.x), I(b.y), I(c.x), I(c.y)).certain_sign();
    return sign ? sign : rational::orientation(exact_value(a), exact_value(b), exact_value(c));
}
// Sign is positive inside for counterclockwise abc, reversed for clockwise abc.
inline int incircle(floating::Point a, floating::Point b, floating::Point c, floating::Point d) {
    for (auto p : {a, b, c, d})
        floating::validate(p);
    using I = detail::Interval;
    const int sign =
        detail::circle(I(a.x), I(a.y), I(b.x), I(b.y), I(c.x), I(c.y), I(d.x), I(d.y)).certain_sign();
    return sign ? sign
                : detail::circle(dyadic(a.x), dyadic(a.y), dyadic(b.x), dyadic(b.y), dyadic(c.x), dyadic(c.y),
                                 dyadic(d.x), dyadic(d.y))
                      .sign();
}
inline int incircle(const rational::Point &a, const rational::Point &b, const rational::Point &c,
                    const rational::Point &d) {
    return detail::circle(a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y).sign();
}
} // namespace cp::geometry::robust
