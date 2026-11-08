#pragma once
#include "minimum_enclosing_circle.hpp"
namespace cp::geometry::floating {
struct TriangleCenters {
    Circle circumcircle, incircle;
    Point centroid, orthocenter;
};
inline std::optional<TriangleCenters> triangle_centers(Point a, Point b, Point c, Tolerance t = Tolerance()) {
    for (auto p : {a, b, c})
        validate(p);
    const auto det = cross(a, b, c);
    if (!t.sign(det))
        return std::nullopt;
    const auto ab = distance(a, b), bc = distance(b, c), ca = distance(c, a),
               perimeter = finite(ab + bc + ca);
    const auto center = a + (b - a) * (ca / perimeter) + (c - a) * (ab / perimeter);
    const auto outer = circle_detail::through_three(a, b, c);
    return TriangleCenters{outer, Circle(center, finite(std::abs(det) / perimeter)),
                           a + (b - a) / 3 + (c - a) / 3, a + (b - outer.center) + (c - outer.center)};
}
inline std::optional<Circle> circumcircle(Point a, Point b, Point c, Tolerance t = Tolerance()) {
    auto v = triangle_centers(a, b, c, t);
    return v ? std::optional<Circle>(v->circumcircle) : std::nullopt;
}
inline std::optional<Circle> incircle(Point a, Point b, Point c, Tolerance t = Tolerance()) {
    auto v = triangle_centers(a, b, c, t);
    return v ? std::optional<Circle>(v->incircle) : std::nullopt;
}
inline std::optional<Point> orthocenter(Point a, Point b, Point c, Tolerance t = Tolerance()) {
    auto v = triangle_centers(a, b, c, t);
    return v ? std::optional<Point>(v->orthocenter) : std::nullopt;
}
} // namespace cp::geometry::floating
