#pragma once
#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>
#include "rectangle.hpp"
#include "../ordered_structure/coordinate_compression.hpp"
#include "../range_query/fenwick_tree.hpp"

// Static weighted point sums in half-open rectangles, returned in query order.
// Empty/reversed rectangles have sum 0; duplicate points accumulate. Weight is a group.
// Docs: docs/offline_query/sweep_line.md
namespace cp {

template <class Coord, class Weight>
std::vector<Weight> static_rectangle_sum(std::vector<WeightedPoint<Coord, Weight>> points,
                                       const std::vector<Rectangle<Coord>>& rectangles) {
    assert(points.size() <= 100000000 && rectangles.size() <= 100000000);
    std::vector<Weight> answer(rectangles.size(), Weight(0));
    if (points.empty() || rectangles.empty()) return answer;
    std::vector<Coord> ys;
    ys.reserve(points.size());
    for (const auto& p : points) ys.push_back(p.y);
    CoordinateCompression<Coord> y(std::move(ys));
    FenwickTree<Weight> bit(y.size());
    struct Event { Coord x; int low, high, id; bool positive; };
    std::vector<Event> events;
    events.reserve(rectangles.size() * 2);
    for (int i = 0; i < int(rectangles.size()); ++i) {
        const auto& q = rectangles[i];
        if (q.empty()) continue;
        int l = y.lower_bound(q.down), r = y.lower_bound(q.up);
        events.push_back({q.left, l, r, i, false});
        events.push_back({q.right, l, r, i, true});
    }
    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) { return a.x < b.x; });
    std::sort(events.begin(), events.end(), [](const auto& a, const auto& b) { return a.x < b.x; });
    int p = 0;
    for (const auto& event : events) {
        while (p < int(points.size()) && points[p].x < event.x) {
            bit.add(y.lower_bound(points[p].y), points[p].weight);
            ++p;
        }
        Weight sum = bit.sum(event.low, event.high);
        answer[event.id] += event.positive ? sum : Weight(0) - sum;
    }
    return answer;
}

}  // namespace cp
