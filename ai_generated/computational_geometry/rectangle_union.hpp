#pragma once
#include "../basic_utility/big_integer.hpp"
#include <algorithm>
#include <cstdint>
#include <vector>
namespace cp::geometry::exact {
struct Rectangle {
    std::int64_t x1, y1, x2, y2;
};
struct RectangleUnionMeasure {
    BigInt area, perimeter;
};
namespace rectangle_detail {
struct CoverTree {
    struct Node {
        int cover = 0, components = 0;
        bool left = false, right = false;
        BigInt length;
    };
    const std::vector<std::int64_t> &y;
    std::vector<Node> tree;
    explicit CoverTree(const std::vector<std::int64_t> &ys) : y(ys), tree(4 * ys.size()) {}
    void add(int ql, int qr, int delta, int v, int l, int r) {
        if (qr <= l || r <= ql)
            return;
        if (ql <= l && r <= qr)
            tree[v].cover += delta;
        else {
            int m = l + (r - l) / 2;
            add(ql, qr, delta, 2 * v, l, m);
            add(ql, qr, delta, 2 * v + 1, m, r);
        }
        auto &n = tree[v];
        if (n.cover) {
            n.length = BigInt(y[r]) - y[l];
            n.components = 1;
            n.left = n.right = true;
        } else if (r - l == 1) {
            n.length = 0;
            n.components = 0;
            n.left = n.right = false;
        } else {
            const auto &a = tree[2 * v];
            const auto &b = tree[2 * v + 1];
            n.length = a.length + b.length;
            n.components = a.components + b.components - (a.right && b.left);
            n.left = a.left;
            n.right = b.right;
        }
    }
};
} // namespace rectangle_detail
// Closed rectangles; zero width/height contributes neither area nor perimeter. O(n log n).
inline RectangleUnionMeasure rectangle_union_measure(const std::vector<Rectangle> &rectangles) {
    struct Event {
        std::int64_t x, y1, y2;
        int delta;
    };
    std::vector<Event> events;
    std::vector<std::int64_t> ys;
    for (auto r : rectangles) {
        if (r.x1 > r.x2 || r.y1 > r.y2)
            throw std::invalid_argument("reversed rectangle");
        if (r.x1 == r.x2 || r.y1 == r.y2)
            continue;
        events.push_back({r.x1, r.y1, r.y2, 1});
        events.push_back({r.x2, r.y1, r.y2, -1});
        ys.push_back(r.y1);
        ys.push_back(r.y2);
    }
    if (events.empty())
        return {};
    std::sort(ys.begin(), ys.end());
    ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
    std::sort(events.begin(), events.end(),
              [](auto a, auto b) { return a.x != b.x ? a.x < b.x : a.delta > b.delta; });
    rectangle_detail::CoverTree tree(ys);
    RectangleUnionMeasure result;
    auto previous = events.front().x;
    for (const auto &e : events) {
        const BigInt dx = BigInt(e.x) - previous;
        result.area += tree.tree[1].length * dx;
        result.perimeter += BigInt(2) * tree.tree[1].components * dx;
        const auto before = tree.tree[1].length;
        const int l = int(std::lower_bound(ys.begin(), ys.end(), e.y1) - ys.begin()),
                  r = int(std::lower_bound(ys.begin(), ys.end(), e.y2) - ys.begin());
        tree.add(l, r, e.delta, 1, 0, int(ys.size()) - 1);
        result.perimeter += abs(tree.tree[1].length - before);
        previous = e.x;
    }
    return result;
}
inline BigInt rectangle_union_area(const std::vector<Rectangle> &r) {
    return rectangle_union_measure(r).area;
}
inline BigInt rectangle_union_perimeter(const std::vector<Rectangle> &r) {
    return rectangle_union_measure(r).perimeter;
}
} // namespace cp::geometry::exact
