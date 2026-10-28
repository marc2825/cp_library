#pragma once
#include "circle_intersection.hpp"
namespace cp::geometry::floating {
inline long double circle_intersection_area(const Circle &a, const Circle &b) {
    validate(a);
    validate(b);
    const auto r = a.radius, s = b.radius, d = distance(a.center, b.center), pi = std::acos(-1.L);
    if (!r || !s || d >= finite(r + s))
        return 0;
    if (d <= std::abs(r - s))
        return finite(pi * std::min(r, s) * std::min(r, s));
    const auto x =
        std::clamp(finite((finite(r * r) - finite(s * s) + finite(d * d)) / finite(2 * d * r)), -1.L, 1.L);
    const auto y =
        std::clamp(finite((finite(s * s) - finite(r * r) + finite(d * d)) / finite(2 * d * s)), -1.L, 1.L);
    return std::max(0.L, finite(r * r * (std::acos(x) - x * std::sqrt(std::max(0.L, 1 - x * x))) +
                                s * s * (std::acos(y) - y * std::sqrt(std::max(0.L, 1 - y * y)))));
}
// Boundary integral over a simple polygon, independent of its orientation.
inline long double circle_polygon_intersection_area(const Circle &circle, const std::vector<Point> &polygon) {
    validate(circle);
    for (auto p : polygon)
        validate(p);
    if (!circle.radius || polygon.size() < 3)
        return 0;
    const auto r2 = finite(circle.radius * circle.radius);
    long double sum = 0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto a = polygon[i] - circle.center, b = polygon[(i + 1) % polygon.size()] - circle.center,
                   v = b - a;
        const auto vv = norm_squared(v);
        if (!vv)
            continue;
        std::vector<long double> ts{0, 1};
        const auto middle = finite(-dot(a, v) / vv);
        const auto closest = a + v * middle;
        const auto height2 = finite((r2 - norm_squared(closest)) / vv);
        if (height2 > 0) {
            const auto h = std::sqrt(height2);
            for (auto q : {middle - h, middle + h})
                if (q > 0 && q < 1)
                    ts.push_back(q);
        }
        std::sort(ts.begin(), ts.end());
        for (std::size_t j = 1; j < ts.size(); ++j) {
            const auto p = a + v * ts[j - 1], q = a + v * ts[j], mid = a + v * ((ts[j - 1] + ts[j]) / 2);
            const auto part =
                norm_squared(mid) <= r2 ? cross(p, q) : finite(r2 * std::atan2(cross(p, q), dot(p, q)));
            sum = finite(sum + part);
        }
    }
    return std::abs(sum) / 2;
}
// O(n^2 log n), integrates exposed CCW arcs. Exact duplicate circles count once.
inline long double circle_union_area(const std::vector<Circle> &circles) {
    for (const auto &c : circles)
        validate(c);
    const auto pi = std::acos(-1.L), tau = 2 * pi;
    // Translate the integral to reduce cancellation for a far-away cluster.
    const Point origin = circles.empty() ? Point{} : circles.front().center;
    long double total = 0;
    for (std::size_t i = 0; i < circles.size(); ++i) {
        const auto &a = circles[i];
        if (!a.radius)
            continue;
        bool hidden = false;
        std::vector<std::pair<long double, long double>> covered;
        for (std::size_t j = 0; j < circles.size(); ++j)
            if (i != j) {
                const auto &b = circles[j];
                if (!b.radius)
                    continue;
                const auto d = distance(a.center, b.center);
                if (a.center == b.center && a.radius == b.radius) {
                    if (j < i) {
                        hidden = true;
                        break;
                    }
                    continue;
                }
                if (finite(d + a.radius) <= b.radius) {
                    hidden = true;
                    break;
                }
                if (d >= finite(a.radius + b.radius) || finite(d + b.radius) <= a.radius)
                    continue;
                const auto ratio = std::clamp(
                    finite((finite(d * d) + finite(a.radius * a.radius) - finite(b.radius * b.radius)) /
                           finite(2 * d * a.radius)),
                    -1.L, 1.L);
                const auto half = std::acos(ratio);
                auto angle = std::atan2(b.center.y - a.center.y, b.center.x - a.center.x);
                if (angle < 0)
                    angle += tau;
                const auto l = angle - half, r = angle + half;
                if (l < 0) {
                    covered.emplace_back(l + tau, tau);
                    covered.emplace_back(0, r);
                } else if (r > tau) {
                    covered.emplace_back(l, tau);
                    covered.emplace_back(0, r - tau);
                } else
                    covered.emplace_back(l, r);
            }
        if (hidden)
            continue;
        std::sort(covered.begin(), covered.end());
        long double end = 0;
        auto arc = [&](long double l, long double r) {
            if (l == 0 && r == tau) {
                total = finite(total + pi * a.radius * a.radius);
                return;
            }
            const auto center = a.center - origin;
            const auto integral =
                finite(a.radius * a.radius * (r - l) + a.radius * (center.x * (std::sin(r) - std::sin(l)) -
                                                                   center.y * (std::cos(r) - std::cos(l))));
            total = finite(total + integral / 2);
        };
        for (auto p : covered) {
            if (p.first > end)
                arc(end, p.first);
            end = std::max(end, p.second);
        }
        if (end < tau)
            arc(end, tau);
    }
    return std::max(0.L, total);
}
} // namespace cp::geometry::floating
