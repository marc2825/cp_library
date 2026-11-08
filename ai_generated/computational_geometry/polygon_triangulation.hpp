#pragma once
#include "rational/kernel.hpp"
#include <array>
#include <deque>
#include <numeric>
namespace cp::geometry::rational {
// Simple ring without holes. Triangles are CCW and refer to original input indices.
inline std::vector<std::array<std::size_t, 3>> triangulate_polygon(const std::vector<Point> &polygon) {
    std::vector<std::size_t> order;
    for (std::size_t i = 0; i < polygon.size(); ++i)
        if (order.empty() || polygon[order.back()] != polygon[i])
            order.push_back(i);
    if (order.size() > 1 && polygon[order.front()] == polygon[order.back()])
        order.pop_back();
    if (order.size() < 3)
        return {};
    bool collinear = true;
    for (auto i : order)
        if (orientation(polygon[order[0]], polygon[order[1]], polygon[i]))
            collinear = false;
    if (collinear)
        return {};
    for (std::size_t i = 0; i < order.size(); ++i)
        for (std::size_t j = 0; j < i; ++j) {
            const auto i1 = (i + 1) % order.size(), j1 = (j + 1) % order.size();
            auto points = segment_intersection({polygon[order[i]], polygon[order[i1]]},
                                               {polygon[order[j]], polygon[order[j1]]});
            if (points.empty())
                continue;
            if ((i1 == j || j1 == i) && points.size() == 1)
                continue;
            throw std::invalid_argument("triangulation needs a simple ring");
        }
    bool changed = true;
    while (changed && order.size() > 2) {
        changed = false;
        for (std::size_t i = 0; i < order.size(); ++i)
            if (on_segment({polygon[order[(i + order.size() - 1) % order.size()]],
                            polygon[order[(i + 1) % order.size()]]},
                           polygon[order[i]])) {
                order.erase(order.begin() + i);
                changed = true;
                break;
            }
    }
    if (order.size() < 3)
        return {};
    std::vector<Point> ring;
    for (auto i : order)
        ring.push_back(polygon[i]);
    if (signed_area2(ring) < 0)
        std::reverse(order.begin(), order.end());
    const auto n = order.size();
    std::vector<std::size_t> previous(n), next(n);
    std::vector<bool> alive(n, true);
    for (std::size_t i = 0; i < n; ++i) {
        previous[i] = (i + n - 1) % n;
        next[i] = (i + 1) % n;
    }
    auto ear = [&](std::size_t i) {
        if (!alive[i])
            return false;
        const auto &a = polygon[order[previous[i]]], &b = polygon[order[i]], &c = polygon[order[next[i]]];
        if (orientation(a, b, c) <= 0)
            return false;
        for (std::size_t j = 0; j < n; ++j)
            if (alive[j] && j != i && j != previous[i] && j != next[i]) {
                const auto &p = polygon[order[j]];
                if (orientation(a, b, p) >= 0 && orientation(b, c, p) >= 0 && orientation(c, a, p) >= 0)
                    return false;
            }
        return true;
    };
    std::deque<std::size_t> ready;
    for (std::size_t i = 0; i < n; ++i)
        if (ear(i))
            ready.push_back(i);
    std::vector<std::array<std::size_t, 3>> out;
    auto remaining = n;
    while (remaining > 3) {
        if (ready.empty())
            throw std::logic_error("simple polygon has no ear");
        const auto i = ready.front();
        ready.pop_front();
        if (!ear(i))
            continue;
        const auto a = previous[i], b = next[i];
        out.push_back({order[a], order[i], order[b]});
        alive[i] = false;
        next[a] = b;
        previous[b] = a;
        --remaining;
        if (ear(a))
            ready.push_back(a);
        if (ear(b))
            ready.push_back(b);
    }
    for (std::size_t i = 0; i < n; ++i)
        if (alive[i]) {
            out.push_back({order[i], order[next[i]], order[next[next[i]]]});
            break;
        }
    return out;
}
} // namespace cp::geometry::rational
