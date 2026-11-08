#pragma once
#include "robust_predicates.hpp"
#include <array>
#include <map>
#include <optional>
namespace cp::geometry::spatial {
struct Point {
    long double x = 0, y = 0, z = 0;
    Point() = default;
    Point(long double a, long double b, long double c) : x(a), y(b), z(c) {
        floating::input(a);
        floating::input(b);
        floating::input(c);
    }
    friend Point operator+(Point a, Point b) {
        return {floating::finite(a.x + b.x), floating::finite(a.y + b.y), floating::finite(a.z + b.z)};
    }
    friend Point operator-(Point a, Point b) {
        return {floating::finite(a.x - b.x), floating::finite(a.y - b.y), floating::finite(a.z - b.z)};
    }
    friend Point operator*(Point a, long double k) {
        floating::input(k);
        return {floating::finite(a.x * k), floating::finite(a.y * k), floating::finite(a.z * k)};
    }
    friend Point operator/(Point a, long double k) {
        floating::input(k);
        if (!k)
            throw std::domain_error("3D point division by zero");
        return a * (1 / k);
    }
    friend bool operator==(Point a, Point b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
};
inline void validate(Point p) {
    floating::input(p.x);
    floating::input(p.y);
    floating::input(p.z);
}
inline long double dot(Point a, Point b) {
    validate(a);
    validate(b);
    return floating::finite(a.x * b.x + a.y * b.y + a.z * b.z);
}
inline Point cross(Point a, Point b) {
    validate(a);
    validate(b);
    return {floating::finite(a.y * b.z - a.z * b.y), floating::finite(a.z * b.x - a.x * b.z),
            floating::finite(a.x * b.y - a.y * b.x)};
}
inline long double norm_squared(Point a) { return dot(a, a); }
inline long double distance(Point a, Point b) { return floating::finite(std::sqrt(norm_squared(a - b))); }
struct Line {
    Point origin, direction;
    Line(Point p, Point d) : origin(p), direction(d) {
        validate(p);
        validate(d);
        if (d == Point{})
            throw std::invalid_argument("3D line needs nonzero direction");
    }
};
struct Segment {
    Point a, b;
};
struct Plane {
    Point normal;
    long double offset;
    Plane(Point n, long double d) : normal(n), offset(d) {
        validate(n);
        floating::input(d);
        if (n == Point{})
            throw std::invalid_argument("plane needs nonzero normal");
    }
};
struct Sphere {
    Point center;
    long double radius;
    Sphere(Point c = {}, long double r = 0) : center(c), radius(r) {
        validate(c);
        floating::input(r);
        if (r < 0)
            throw std::invalid_argument("negative sphere radius");
    }
};
inline void validate(const Line &l) {
    validate(l.origin);
    validate(l.direction);
    if (l.direction == Point{})
        throw std::invalid_argument("zero line direction");
}
inline void validate(const Plane &p) {
    validate(p.normal);
    floating::input(p.offset);
    if (p.normal == Point{})
        throw std::invalid_argument("zero plane normal");
}
inline void validate(const Sphere &s) {
    validate(s.center);
    floating::input(s.radius);
    if (s.radius < 0)
        throw std::invalid_argument("negative sphere radius");
}
inline Plane plane_through(Point a, Point b, Point c) {
    auto n = cross(b - a, c - a);
    return Plane(n, dot(n, a));
}
inline Point projection(const Line &l, Point p) {
    validate(l);
    return l.origin + l.direction * (dot(p - l.origin, l.direction) / norm_squared(l.direction));
}
inline Point projection(const Plane &plane, Point p) {
    validate(plane);
    return p - plane.normal * ((dot(plane.normal, p) - plane.offset) / norm_squared(plane.normal));
}
inline long double distance(const Line &l, Point p) { return distance(projection(l, p), p); }
inline long double distance(const Plane &plane, Point p) { return distance(projection(plane, p), p); }
inline long double distance(Segment s, Point p) {
    validate(s.a);
    validate(s.b);
    if (s.a == s.b)
        return distance(s.a, p);
    const auto v = s.b - s.a;
    return distance(s.a + v * std::clamp(dot(p - s.a, v) / norm_squared(v), 0.L, 1.L), p);
}
inline long double distance(const Line &a, const Line &b) {
    validate(a);
    validate(b);
    const auto n = cross(a.direction, b.direction);
    const auto norm = norm_squared(n);
    return norm ? floating::finite(std::abs(dot(b.origin - a.origin, n)) / std::sqrt(norm))
                : distance(a, b.origin);
}
struct LinePlaneIntersection {
    LineRelation relation;
    std::optional<Point> point;
};
inline LinePlaneIntersection intersection(const Line &l, const Plane &p,
                                          floating::Tolerance tolerance = floating::Tolerance()) {
    validate(l);
    validate(p);
    const auto divisor = dot(p.normal, l.direction),
               numerator = floating::finite(p.offset - dot(p.normal, l.origin));
    if (!tolerance.sign(divisor))
        return {tolerance.sign(numerator) ? LineRelation::parallel : LineRelation::coincident, std::nullopt};
    return {LineRelation::unique, l.origin + l.direction * (numerator / divisor)};
}
struct PlaneIntersection {
    LineRelation relation;
    std::optional<Line> line;
};
inline PlaneIntersection intersection(const Plane &a, const Plane &b,
                                      floating::Tolerance t = floating::Tolerance()) {
    validate(a);
    validate(b);
    const auto d = cross(a.normal, b.normal);
    const auto square = norm_squared(d);
    if (!t.sign(square)) {
        const auto p = a.normal * (a.offset / norm_squared(a.normal));
        return {t.sign(dot(b.normal, p) - b.offset) ? LineRelation::parallel : LineRelation::coincident,
                std::nullopt};
    }
    const auto origin = (cross(b.normal, d) * a.offset + cross(d, a.normal) * b.offset) / square;
    return {LineRelation::unique, Line(origin, d)};
}
inline std::vector<Point> intersection(const Line &line, const Sphere &sphere,
                                       floating::Tolerance t = floating::Tolerance()) {
    validate(line);
    validate(sphere);
    const auto center = projection(line, sphere.center);
    const auto h = floating::finite(sphere.radius * sphere.radius - norm_squared(center - sphere.center));
    if (t.sign(h) < 0)
        return {};
    if (!t.sign(h))
        return {center};
    const auto delta = line.direction * std::sqrt(h / norm_squared(line.direction));
    return {center - delta, center + delta};
}
struct Circle {
    Point center, normal;
    long double radius;
};
inline std::optional<Circle> intersection(const Plane &plane, const Sphere &sphere,
                                          floating::Tolerance t = floating::Tolerance()) {
    validate(plane);
    validate(sphere);
    const auto center = projection(plane, sphere.center);
    const auto h = floating::finite(sphere.radius * sphere.radius - norm_squared(center - sphere.center));
    if (t.sign(h) < 0)
        return std::nullopt;
    return Circle{center, plane.normal, std::sqrt(std::max(0.L, h))};
}
struct SphereIntersection {
    bool coincident = false;
    std::optional<Circle> circle;
};
inline SphereIntersection intersection(const Sphere &a, const Sphere &b,
                                       floating::Tolerance t = floating::Tolerance()) {
    validate(a);
    validate(b);
    const auto d = distance(a.center, b.center);
    if (!d) {
        if (a.radius != b.radius)
            return {};
        return a.radius ? SphereIntersection{true, std::nullopt}
                        : SphereIntersection{false, Circle{a.center, {1, 0, 0}, 0}};
    }
    if (t.sign(d - a.radius - b.radius) > 0 || t.sign(d - std::abs(a.radius - b.radius)) < 0)
        return {};
    const auto along = floating::finite((a.radius * a.radius - b.radius * b.radius + d * d) / (2 * d));
    const auto h = floating::finite(a.radius * a.radius - along * along);
    return {false, Circle{a.center + (b.center - a.center) * (along / d), b.center - a.center,
                          std::sqrt(std::max(0.L, h))}};
}
namespace exact {
using Scalar = BigRational;
struct Point {
    Scalar x, y, z;
    friend Point operator+(const Point &a, const Point &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
    friend Point operator-(const Point &a, const Point &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
    friend Point operator/(const Point &a, const Scalar &k) { return {a.x / k, a.y / k, a.z / k}; }
    friend bool operator==(const Point &a, const Point &b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
    friend bool operator<(const Point &a, const Point &b) {
        return a.x != b.x ? a.x < b.x : a.y != b.y ? a.y < b.y : a.z < b.z;
    }
};
inline Scalar dot(const Point &a, const Point &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Point cross(const Point &a, const Point &b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Scalar determinant(const Point &a, const Point &b, const Point &c, const Point &d) {
    return dot(cross(b - a, c - a), d - a);
}
inline int orientation(const Point &a, const Point &b, const Point &c, const Point &d) {
    return determinant(a, b, c, d).sign();
}
struct ConvexHull {
    int dimension = -1;
    std::vector<Point> points;
    std::vector<std::array<std::size_t, 3>> faces;
    std::vector<std::size_t> boundary;
    Scalar volume() const {
        if (dimension < 3)
            return 0;
        Scalar sum;
        for (auto f : faces)
            sum += dot(points[f[0]], cross(points[f[1]], points[f[2]]));
        return abs(sum) / 6;
    }
};
// Incremental exact hull, O(n^2 log n) arithmetic operations; supports lower dimensions.
inline ConvexHull convex_hull(std::vector<Point> points) {
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    ConvexHull out;
    out.points = points;
    const auto n = points.size();
    if (!n)
        return out;
    out.dimension = 0;
    out.boundary = {0};
    if (n == 1)
        return out;
    std::size_t third = 2;
    while (third < n && cross(points[1] - points[0], points[third] - points[0]) == Point{})
        ++third;
    if (third == n) {
        out.dimension = 1;
        out.boundary = {0, n - 1};
        return out;
    }
    std::size_t fourth = 1;
    while (fourth < n && !orientation(points[0], points[1], points[third], points[fourth]))
        ++fourth;
    if (fourth == n) {
        out.dimension = 2;
        const auto normal = cross(points[1] - points[0], points[third] - points[0]);
        std::map<rational::Point, std::size_t> ids;
        std::vector<rational::Point> projection;
        for (std::size_t i = 0; i < n; ++i) {
            rational::Point p = normal.x   ? rational::Point{points[i].y, points[i].z}
                                : normal.y ? rational::Point{points[i].z, points[i].x}
                                           : rational::Point{points[i].x, points[i].y};
            ids[p] = i;
            projection.push_back(p);
        }
        out.boundary.clear();
        for (const auto &p : rational::convex_hull(std::move(projection)))
            out.boundary.push_back(ids.at(p));
        for (std::size_t i = 1; i + 1 < out.boundary.size(); ++i)
            out.faces.push_back({out.boundary[0], out.boundary[i], out.boundary[i + 1]});
        return out;
    }
    out.dimension = 3;
    out.boundary.clear();
    const auto inside = (points[0] + points[1] + points[third] + points[fourth]) / 4;
    auto add = [&](std::size_t a, std::size_t b, std::size_t c) {
        if (cross(points[b] - points[a], points[c] - points[a]) == Point{})
            return;
        if (orientation(points[a], points[b], points[c], inside) > 0)
            std::swap(b, c);
        out.faces.push_back({a, b, c});
    };
    add(0, 1, third);
    add(0, fourth, 1);
    add(0, third, fourth);
    add(1, fourth, third);
    for (std::size_t p = 0; p < n; ++p)
        if (p != 0 && p != 1 && p != third && p != fourth) {
            std::vector<std::array<std::size_t, 3>> kept;
            std::map<std::pair<std::size_t, std::size_t>, int> horizon;
            for (auto face : out.faces) {
                if (orientation(points[face[0]], points[face[1]], points[face[2]], points[p]) <= 0) {
                    kept.push_back(face);
                    continue;
                }
                for (int j = 0; j < 3; ++j) {
                    const auto edge = std::make_pair(face[j], face[(j + 1) % 3]),
                               reverse = std::make_pair(edge.second, edge.first);
                    auto it = horizon.find(reverse);
                    if (it != horizon.end())
                        horizon.erase(it);
                    else
                        horizon[edge] = 1;
                }
            }
            out.faces = std::move(kept);
            for (const auto &edge : horizon)
                add(edge.first.first, edge.first.second, p);
        }
    return out;
}
} // namespace exact
inline exact::Point exact_value(Point p) {
    return {robust::dyadic(p.x), robust::dyadic(p.y), robust::dyadic(p.z)};
}
inline exact::ConvexHull convex_hull(const std::vector<Point> &points) {
    std::vector<exact::Point> exact;
    for (auto p : points)
        exact.push_back(exact_value(p));
    return exact::convex_hull(std::move(exact));
}
} // namespace cp::geometry::spatial
