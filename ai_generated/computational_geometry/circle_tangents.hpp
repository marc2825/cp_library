#pragma once
#include "circle_intersection.hpp"
namespace cp::geometry::floating {
struct CommonTangent {
    Point normal;
    long double offset;
    Point first, second;
};
struct CircleTangents {
    bool infinite = false;
    std::vector<CommonTangent> lines;
};
// Lines use unit normal dot(x)=offset; contact points may coincide at tangencies.
inline CircleTangents common_tangents(const Circle &a, const Circle &b, Tolerance t = Tolerance()) {
    validate(a);
    validate(b);
    const auto delta = b.center - a.center;
    const auto d = distance(a.center, b.center);
    if (!d)
        return {a.radius == b.radius, {}};
    CircleTangents out;
    const auto u = delta / d;
    for (int side : {1, -1}) {
        const auto h = finite((a.radius - side * b.radius) / d);
        if (t.sign(std::abs(h) - 1) > 0)
            continue;
        const auto bounded = std::clamp(h, -1.L, 1.L),
                   height = std::sqrt(std::max(0.L, 1 - bounded * bounded));
        for (int direction : {1, -1}) {
            const auto n = u * bounded + Point{-u.y, u.x} * (direction * height);
            const auto p = a.center + n * a.radius, q = b.center + n * (side * b.radius);
            auto normal = n;
            auto offset = finite(dot(n, a.center) + a.radius);
            if (normal.x < 0 || (normal.x == 0 && normal.y < 0)) {
                normal = normal * (-1);
                offset = -offset;
            }
            bool duplicate = false;
            for (const auto &line : out.lines)
                if (almost_equal(line.normal, normal, t) && !t.sign(line.offset - offset))
                    duplicate = true;
            if (!duplicate)
                out.lines.push_back({normal, offset, p, q});
        }
    }
    std::sort(out.lines.begin(), out.lines.end(), [](const auto &x, const auto &y) {
        return x.normal != y.normal ? x.normal < y.normal : x.offset < y.offset;
    });
    return out;
}
inline CircleTangents tangents_from_point(const Circle &circle, Point p, Tolerance t = Tolerance()) {
    return common_tangents(Circle(p, 0), circle, t);
}
} // namespace cp::geometry::floating
