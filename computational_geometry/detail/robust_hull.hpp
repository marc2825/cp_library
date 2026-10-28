#pragma once
#include "../exact/primitives.hpp"
#include "../robust_predicates.hpp"
#include "algorithms.hpp"
namespace cp::geometry::robust {
struct IntegerKernel {
    using P = exact::Point;
    void validate(P) const {}
    int turn(P a, P b, P c) const { return robust::orientation(a, b, c); }
    bool same(P a, P b) const { return a == b; }
    bool on(exact::Segment s, P p) const {
        return !turn(s.a, s.b, p) && std::min(s.a.x, s.b.x) <= p.x && p.x <= std::max(s.a.x, s.b.x) &&
               std::min(s.a.y, s.b.y) <= p.y && p.y <= std::max(s.a.y, s.b.y);
    }
};
inline std::vector<exact::Point> convex_hull(std::vector<exact::Point> points) {
    return geometry::detail::hull(std::move(points), false, IntegerKernel{});
}
inline BigInt squared_distance(exact::Point a, exact::Point b) {
    const BigInt x = BigInt(a.x) - b.x, y = BigInt(a.y) - b.y;
    return x * x + y * y;
}
} // namespace cp::geometry::robust
