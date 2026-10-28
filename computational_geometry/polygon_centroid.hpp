#pragma once
#include "rational/kernel.hpp"
#include <optional>
namespace cp::geometry {
namespace rational {
inline std::optional<Point> polygon_centroid(const std::vector<Point> &p) {
    Scalar area;
    Point sum;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto &a = p[i];
        const auto &b = p[(i + 1) % p.size()];
        const auto w = cross(a, b);
        area += w;
        sum = sum + (a + b) * w;
    }
    return area ? std::optional<Point>(sum / (3 * area)) : std::nullopt;
}
} // namespace rational
namespace exact {
inline std::optional<rational::Point> polygon_centroid(const std::vector<Point> &p) {
    std::vector<rational::Point> q;
    for (auto a : p)
        q.emplace_back(a);
    return rational::polygon_centroid(q);
}
} // namespace exact
namespace floating {
inline std::optional<Point> polygon_centroid(const std::vector<Point> &p, Tolerance t = Tolerance()) {
    for (auto a : p)
        validate(a);
    if (p.empty())
        return std::nullopt;
    long double area = 0;
    Point sum;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto a = p[i] - p[0], b = p[(i + 1) % p.size()] - p[0];
        const auto w = cross(a, b);
        area = finite(area + w);
        sum = sum + (a + b) * w;
    }
    return t.sign(area) ? std::optional<Point>(p[0] + sum / finite(3 * area)) : std::nullopt;
}
} // namespace floating
} // namespace cp::geometry
