#pragma once
#include "../../basic_utility/big_rational.hpp"
#include "../../basic_utility/binary_rational.hpp"
#include "../exact/point.hpp"
#include "../floating/point.hpp"
#include <algorithm>
#include <vector>

namespace cp::geometry::rational {
using Scalar = BigRational;
struct Point {
    Scalar x, y;
    Point() = default;
    Point(Scalar x_, Scalar y_) : x(std::move(x_)), y(std::move(y_)) {}
    explicit Point(exact::Point p) : x(p.x), y(p.y) {}
    explicit Point(const exact::RationalPoint &p)
        : x(BigInt(p.x.numerator()), BigInt(p.x.denominator())),
          y(BigInt(p.y.numerator()), BigInt(p.y.denominator())) {}
    friend Point operator+(const Point &a, const Point &b) { return {a.x + b.x, a.y + b.y}; }
    friend Point operator-(const Point &a, const Point &b) { return {a.x - b.x, a.y - b.y}; }
    friend Point operator*(const Point &a, const Scalar &k) { return {a.x * k, a.y * k}; }
    friend Point operator/(const Point &a, const Scalar &k) { return {a.x / k, a.y / k}; }
    friend bool operator==(const Point &a, const Point &b) { return a.x == b.x && a.y == b.y; }
    friend bool operator!=(const Point &a, const Point &b) { return !(a == b); }
    friend bool operator<(const Point &a, const Point &b) { return a.x != b.x ? a.x < b.x : a.y < b.y; }
};
struct Segment {
    Point a, b;
};
inline Scalar dot(const Point &a, const Point &b) { return a.x * b.x + a.y * b.y; }
inline Scalar cross(const Point &a, const Point &b) { return a.x * b.y - a.y * b.x; }
inline Scalar cross(const Point &a, const Point &b, const Point &c) { return cross(b - a, c - a); }
inline int orientation(const Point &a, const Point &b, const Point &c) { return cross(a, b, c).sign(); }
inline Scalar squared_distance(const Point &a, const Point &b) { return dot(a - b, a - b); }
inline bool on_segment(const Segment &s, const Point &p) {
    return !orientation(s.a, s.b, p) && dot(p - s.a, p - s.b) <= 0;
}
inline std::vector<Point> segment_intersection(const Segment &a, const Segment &b) {
    const auto u = a.b - a.a, v = b.b - b.a;
    const auto d = cross(u, v);
    if (d) {
        const auto t = cross(b.a - a.a, v) / d, s = cross(b.a - a.a, u) / d;
        return t >= 0 && t <= 1 && s >= 0 && s <= 1 ? std::vector<Point>{a.a + u * t} : std::vector<Point>{};
    }
    std::vector<Point> out;
    for (const auto &p : {a.a, a.b, b.a, b.b})
        if (on_segment(a, p) && on_segment(b, p))
            out.push_back(p);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    if (out.size() > 2)
        out = {out.front(), out.back()};
    return out;
}
inline Scalar signed_area2(const std::vector<Point> &p) {
    Scalar a;
    for (std::size_t i = 0; i < p.size(); ++i)
        a += cross(p[i], p[(i + 1) % p.size()]);
    return a;
}
inline Location contains(const std::vector<Point> &p, const Point &q) {
    bool inside = false;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto &a = p[i];
        const auto &b = p[(i + 1) % p.size()];
        if (on_segment({a, b}, q))
            return Location::boundary;
        if ((a.y > q.y) != (b.y > q.y))
            if ((orientation(a, b, q) > 0) == (b.y > a.y))
                inside = !inside;
    }
    return inside ? Location::inside : Location::outside;
}
inline std::vector<Point> convex_hull(std::vector<Point> p) {
    std::sort(p.begin(), p.end());
    p.erase(std::unique(p.begin(), p.end()), p.end());
    if (p.size() < 3)
        return p;
    std::vector<Point> out;
    for (const auto &q : p) {
        while (out.size() > 1 && orientation(out[out.size() - 2], out.back(), q) <= 0)
            out.pop_back();
        out.push_back(q);
    }
    const auto lower = out.size();
    for (std::size_t i = p.size() - 1; i-- > 0;) {
        while (out.size() > lower && orientation(out[out.size() - 2], out.back(), p[i]) <= 0)
            out.pop_back();
        out.push_back(p[i]);
    }
    out.pop_back();
    return out;
}
// Keep cross(direction, p-origin) >= 0; point/segment polygons are supported.
inline std::vector<Point> convex_cut(const std::vector<Point> &p, const Point &origin,
                                     const Point &direction) {
    if (direction == Point{})
        throw std::invalid_argument("cut needs nonzero direction");
    std::vector<Point> out;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto &a = p[i];
        const auto &b = p[(i + 1) % p.size()];
        const auto u = cross(direction, a - origin), v = cross(direction, b - origin);
        if (u >= 0)
            out.push_back(a);
        if (u.sign() * v.sign() < 0)
            out.push_back(a + (b - a) * (u / (u - v)));
    }
    out.erase(std::unique(out.begin(), out.end()), out.end());
    if (out.size() > 1 && out.front() == out.back())
        out.pop_back();
    return out;
}
inline BigInt floor(const Scalar &x) {
    auto q = divmod(x.numerator(), x.denominator());
    return q.first - (q.second.sign() < 0 ? 1 : 0);
}
inline BigInt ceil(const Scalar &x) { return -floor(-x); }
inline long double approximate(const Scalar &x) { return rational_to_long_double(x); }
inline floating::Point approximate(const Point &p) { return {approximate(p.x), approximate(p.y)}; }
} // namespace cp::geometry::rational
