#pragma once
#include "rational/kernel.hpp"
#include <map>
#include <numeric>
namespace cp::geometry::rational {
struct SegmentCrossing {
    std::size_t first, second;
    std::vector<Point> points;
};
namespace arrangement_detail {
template <class Callback>
void sweep(const std::vector<Segment> &segments, Callback callback, std::size_t max_candidates) {
    std::vector<std::size_t> order(segments.size()), active;
    std::iota(order.begin(), order.end(), 0);
    const auto low = [&](std::size_t i) { return std::min(segments[i].a.x, segments[i].b.x); };
    const auto high = [&](std::size_t i) { return std::max(segments[i].a.x, segments[i].b.x); };
    std::sort(order.begin(), order.end(),
              [&](auto i, auto j) { return low(i) != low(j) ? low(i) < low(j) : i < j; });
    std::size_t candidates = 0;
    for (auto i : order) {
        active.erase(std::remove_if(active.begin(), active.end(), [&](auto j) { return high(j) < low(i); }),
                     active.end());
        for (auto j : active) {
            if (candidates++ >= max_candidates)
                throw std::length_error("segment sweep candidate budget exceeded");
            const auto &a = segments[i];
            const auto &b = segments[j];
            if (std::max(std::min(a.a.y, a.b.y), std::min(b.a.y, b.b.y)) >
                std::min(std::max(a.a.y, a.b.y), std::max(b.a.y, b.b.y)))
                continue;
            auto points = segment_intersection(a, b);
            if (!points.empty() && !callback(std::min(i, j), std::max(i, j), std::move(points)))
                return;
        }
        active.push_back(i);
    }
}
} // namespace arrangement_detail
// Bounding-box sweep, not Bentley-Ottmann: O(n log n + candidate pairs), worst O(n^2).
inline std::vector<SegmentCrossing> segment_intersections(const std::vector<Segment> &s,
                                                          std::size_t max_pairs = 2000000,
                                                          std::size_t max_candidates = 20000000) {
    std::vector<SegmentCrossing> out;
    arrangement_detail::sweep(
        s,
        [&](auto i, auto j, auto points) {
            if (out.size() == max_pairs)
                throw std::length_error("intersection output budget exceeded");
            out.push_back({i, j, std::move(points)});
            return true;
        },
        max_candidates);
    std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
        return std::make_pair(a.first, a.second) < std::make_pair(b.first, b.second);
    });
    return out;
}
inline bool any_segment_intersection(const std::vector<Segment> &s, std::size_t max_candidates = 20000000) {
    bool found = false;
    arrangement_detail::sweep(
        s,
        [&](auto, auto, auto) {
            found = true;
            return false;
        },
        max_candidates);
    return found;
}
struct SegmentArrangement {
    struct Edge {
        std::size_t from, to;
        std::vector<std::size_t> segments;
    };
    std::vector<Point> vertices;
    std::vector<Edge> edges;
};
inline SegmentArrangement segment_arrangement(const std::vector<Segment> &segments,
                                              std::size_t max_pairs = 2000000,
                                              std::size_t max_candidates = 20000000) {
    std::vector<std::vector<Point>> cuts;
    for (const auto &s : segments)
        cuts.push_back({s.a, s.b});
    for (const auto &crossing : segment_intersections(segments, max_pairs, max_candidates))
        for (auto i : {crossing.first, crossing.second})
            cuts[i].insert(cuts[i].end(), crossing.points.begin(), crossing.points.end());
    std::map<Point, std::size_t> ids;
    for (auto &points : cuts) {
        std::sort(points.begin(), points.end());
        points.erase(std::unique(points.begin(), points.end()), points.end());
        for (const auto &p : points)
            ids.emplace(p, 0);
    }
    SegmentArrangement out;
    for (auto &entry : ids) {
        entry.second = out.vertices.size();
        out.vertices.push_back(entry.first);
    }
    std::map<std::pair<std::size_t, std::size_t>, std::vector<std::size_t>> edges;
    for (std::size_t i = 0; i < cuts.size(); ++i)
        for (std::size_t j = 1; j < cuts[i].size(); ++j)
            edges[{ids.at(cuts[i][j - 1]), ids.at(cuts[i][j])}].push_back(i);
    for (auto &entry : edges)
        out.edges.push_back({entry.first.first, entry.first.second, std::move(entry.second)});
    return out;
}
} // namespace cp::geometry::rational
