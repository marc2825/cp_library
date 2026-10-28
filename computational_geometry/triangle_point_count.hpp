#pragma once
#include "robust_predicates.hpp"
#include <array>
namespace cp::geometry::exact {
// Static kd-tree; duplicates count separately. O(n log n) build, worst O(n) per triangle.
class TrianglePointCounter {
    struct Node {
        std::size_t begin, end;
        int left = -1, right = -1;
        Point low, high;
    };
    std::vector<Point> points_;
    std::vector<Node> nodes_;
    int build(std::size_t l, std::size_t r) {
        const int id = int(nodes_.size());
        Point lo = points_[l], hi = lo;
        for (auto i = l; i < r; ++i) {
            lo.x = std::min(lo.x, points_[i].x);
            lo.y = std::min(lo.y, points_[i].y);
            hi.x = std::max(hi.x, points_[i].x);
            hi.y = std::max(hi.y, points_[i].y);
        }
        nodes_.push_back({l, r, -1, -1, lo, hi});
        if (r - l > 16) {
            const bool x = (BigInt(hi.x) - lo.x) > (BigInt(hi.y) - lo.y);
            const auto m = l + (r - l) / 2;
            std::nth_element(
                points_.begin() + l, points_.begin() + m, points_.begin() + r, [&](auto a, auto b) {
                    return x ? (a.x != b.x ? a.x < b.x : a.y < b.y) : (a.y != b.y ? a.y < b.y : a.x < b.x);
                });
            const int a = build(l, m), b = build(m, r);
            nodes_[id].left = a;
            nodes_[id].right = b;
        }
        return id;
    }
    std::size_t query(int v, std::array<Point, 3> triangle, bool boundary, bool degenerate) const {
        const auto &node = nodes_[v];
        const std::array<Point, 4> box{node.low, Point{node.high.x, node.low.y}, node.high,
                                       Point{node.low.x, node.high.y}};
        bool full = true;
        if (!degenerate) {
            for (int i = 0; i < 3; ++i) {
                int smallest = 1, largest = -1;
                for (auto p : box) {
                    const int s = robust::orientation(triangle[i], triangle[(i + 1) % 3], p);
                    smallest = std::min(smallest, s);
                    largest = std::max(largest, s);
                }
                if (boundary ? largest < 0 : largest <= 0)
                    return 0;
                if (boundary ? smallest < 0 : smallest <= 0)
                    full = false;
            }
            if (full)
                return node.end - node.begin;
        } else {
            if (!boundary)
                return 0;
            auto lo = std::min({triangle[0], triangle[1], triangle[2]}),
                 hi = std::max({triangle[0], triangle[1], triangle[2]});
            if (node.high.x < std::min(lo.x, hi.x) || node.low.x > std::max(lo.x, hi.x) ||
                node.high.y < std::min(lo.y, hi.y) || node.low.y > std::max(lo.y, hi.y))
                return 0;
        }
        if (node.left >= 0)
            return query(node.left, triangle, boundary, degenerate) +
                   query(node.right, triangle, boundary, degenerate);
        std::size_t result = 0;
        for (auto i = node.begin; i < node.end; ++i) {
            auto p = points_[i];
            bool inside = true;
            if (degenerate) {
                auto lo = std::min({triangle[0], triangle[1], triangle[2]}),
                     hi = std::max({triangle[0], triangle[1], triangle[2]});
                inside = !robust::orientation(lo, hi, p) && std::min(lo.x, hi.x) <= p.x &&
                         p.x <= std::max(lo.x, hi.x) && std::min(lo.y, hi.y) <= p.y &&
                         p.y <= std::max(lo.y, hi.y);
            } else
                for (int j = 0; j < 3; ++j) {
                    const int side = robust::orientation(triangle[j], triangle[(j + 1) % 3], p);
                    inside &= boundary ? side >= 0 : side > 0;
                }
            result += inside;
        }
        return result;
    }

  public:
    explicit TrianglePointCounter(std::vector<Point> points = {}) : points_(std::move(points)) {
        if (points_.size() > std::size_t(std::numeric_limits<int>::max() / 2))
            throw std::length_error("too many triangle query points");
        if (!points_.empty())
            build(0, points_.size());
    }
    std::size_t size() const { return points_.size(); }
    std::size_t count(Point a, Point b, Point c, bool include_boundary = true) const {
        if (nodes_.empty())
            return 0;
        const int s = robust::orientation(a, b, c);
        if (s < 0)
            std::swap(b, c);
        return query(0, {a, b, c}, include_boundary, s == 0);
    }
};
} // namespace cp::geometry::exact
