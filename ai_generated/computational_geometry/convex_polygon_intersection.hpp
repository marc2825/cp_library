#pragma once
#include "rational/kernel.hpp"
namespace cp::geometry::rational {
// Canonicalizes convex vertex sets. Handles empty/point/segment intersections exactly.
inline std::vector<Point> convex_polygon_intersection(std::vector<Point> a, std::vector<Point> b) {
    a = convex_hull(std::move(a));
    b = convex_hull(std::move(b));
    if (a.empty() || b.empty())
        return {};
    if (b.size() < 3) {
        if (a.size() < b.size())
            std::swap(a, b);
        std::vector<Point> candidates;
        for (const auto &p : b)
            if (contains(a, p) != Location::outside)
                candidates.push_back(p);
        for (const auto &p : a)
            if (contains(b, p) != Location::outside)
                candidates.push_back(p);
        for (std::size_t i = 0; i < a.size(); ++i)
            for (std::size_t j = 0; j < b.size(); ++j) {
                auto q = segment_intersection({a[i], a[(i + 1) % a.size()]}, {b[j], b[(j + 1) % b.size()]});
                candidates.insert(candidates.end(), q.begin(), q.end());
            }
        return convex_hull(std::move(candidates));
    }
    for (std::size_t i = 0; i < b.size() && !a.empty(); ++i)
        a = convex_cut(a, b[i], b[(i + 1) % b.size()] - b[i]);
    return convex_hull(std::move(a));
}
} // namespace cp::geometry::rational
