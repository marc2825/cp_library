#pragma once
#include "detail/robust_hull.hpp"
#include <map>
namespace cp::geometry::exact {
struct FarthestNeighbor {
    std::size_t index;
    BigInt squared_distance;
};
// Distinct points in strict convex position (any input order). One maximizer per input vertex.
inline std::vector<FarthestNeighbor> convex_farthest_neighbors(const std::vector<Point> &vertices) {
    const auto hull = robust::convex_hull(vertices);
    const auto n = hull.size();
    if (n != vertices.size())
        throw std::invalid_argument("vertices must be distinct and strict convex");
    if (!n)
        return {};
    std::map<Point, std::size_t> ids;
    for (std::size_t i = 0; i < n; ++i)
        ids[vertices[i]] = i;
    std::vector<std::size_t> far(n);
    std::size_t anchor = 0;
    for (std::size_t j = 1; j < n; ++j)
        if (robust::squared_distance(hull[0], hull[j]) >= robust::squared_distance(hull[0], hull[anchor]))
            anchor = j;
    far[0] = anchor;
    auto search = [&](auto &&self, std::size_t l, std::size_t r, std::size_t low, std::size_t high) -> void {
        if (l >= r)
            return;
        const auto mid = l + (r - l) / 2, begin = std::max(low, mid), end = std::min(high, mid + n - 1);
        auto best = begin;
        for (auto j = begin + 1; j <= end; ++j)
            if (robust::squared_distance(hull[mid], hull[j % n]) >=
                robust::squared_distance(hull[mid], hull[best % n]))
                best = j;
        far[mid] = best % n;
        self(self, l, mid, low, best);
        self(self, mid + 1, r, best, high);
    };
    search(search, 1, n, anchor, anchor + n);
    std::vector<FarthestNeighbor> out(n);
    for (std::size_t i = 0; i < n; ++i)
        out[ids.at(hull[i])] = {ids.at(hull[far[i]]), robust::squared_distance(hull[i], hull[far[i]])};
    return out;
}
} // namespace cp::geometry::exact
