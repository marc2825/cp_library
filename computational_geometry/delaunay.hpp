#pragma once
#include "robust_predicates.hpp"
#include <array>
#include <map>
#include <set>
namespace cp::geometry::rational {
struct DelaunayTriangulation {
    std::vector<Point> vertices; // sorted unique sites
    std::vector<std::size_t> original_to_vertex;
    std::vector<std::array<std::size_t, 3>> triangles; // CCW; collinear input has none
    std::vector<std::pair<std::size_t, std::size_t>> edges;
};
// Exact insertion and Lawson flips; no artificial enclosing triangle or epsilon.
inline DelaunayTriangulation delaunay_triangulation(std::vector<Point> input) {
    DelaunayTriangulation out;
    out.vertices = input;
    auto &points = out.vertices;
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    std::map<Point, std::size_t> ids;
    for (std::size_t i = 0; i < points.size(); ++i)
        ids[points[i]] = i;
    for (const auto &p : input)
        out.original_to_vertex.push_back(ids.at(p));
    const auto hull = convex_hull(points);
    if (hull.size() < 3) {
        for (std::size_t i = 1; i < points.size(); ++i)
            out.edges.emplace_back(i - 1, i);
        return out;
    }
    using Edge = std::pair<std::size_t, std::size_t>;
    using Triangle = std::array<std::size_t, 3>;
    std::vector<Triangle> faces;
    std::vector<bool> active;
    std::vector<std::size_t> free;
    std::map<Edge, std::vector<std::size_t>> adjacent;
    std::set<Edge> pending;
    auto key = [](std::size_t a, std::size_t b) -> Edge { return std::minmax(a, b); };
    auto add = [&](Triangle t) {
        const int turn = orientation(points[t[0]], points[t[1]], points[t[2]]);
        if (!turn)
            throw std::logic_error("degenerate Delaunay triangle");
        if (turn < 0)
            std::swap(t[1], t[2]);
        std::size_t id;
        if (free.empty()) {
            id = faces.size();
            faces.push_back(t);
            active.push_back(true);
        } else {
            id = free.back();
            free.pop_back();
            faces[id] = t;
            active[id] = true;
        }
        for (int i = 0; i < 3; ++i) {
            const auto edge = key(t[i], t[(i + 1) % 3]);
            adjacent[edge].push_back(id);
            pending.insert(edge);
        }
    };
    auto remove = [&](std::size_t id) {
        active[id] = false;
        free.push_back(id);
        const auto t = faces[id];
        for (int i = 0; i < 3; ++i) {
            const auto edge = key(t[i], t[(i + 1) % 3]);
            auto it = adjacent.find(edge);
            auto &list = it->second;
            list.erase(std::find(list.begin(), list.end(), id));
            if (list.empty()) {
                adjacent.erase(it);
                pending.erase(edge);
            }
        }
    };
    auto legalize = [&]() {
        while (!pending.empty()) {
            const auto edge = *pending.begin();
            pending.erase(pending.begin());
            const auto it = adjacent.find(edge);
            if (it == adjacent.end() || it->second.size() != 2)
                continue;
            const auto f = it->second[0], g = it->second[1], u = edge.first, v = edge.second;
            std::size_t a = u, b = u;
            for (auto x : faces[f])
                if (x != u && x != v)
                    a = x;
            for (auto x : faces[g])
                if (x != u && x != v)
                    b = x;
            if (orientation(points[a], points[b], points[u]) * orientation(points[a], points[b], points[v]) >=
                0)
                continue;
            const auto t = faces[f];
            if (robust::incircle(points[t[0]], points[t[1]], points[t[2]], points[b]) <= 0)
                continue;
            remove(f);
            remove(g);
            add({a, b, u});
            add({b, a, v});
        }
    };
    std::vector<bool> on_hull(points.size());
    for (const auto &p : hull)
        on_hull[ids.at(p)] = true;
    for (std::size_t i = 1; i + 1 < hull.size(); ++i)
        add({ids.at(hull[0]), ids.at(hull[i]), ids.at(hull[i + 1])});
    legalize();
    for (std::size_t p = 0; p < points.size(); ++p)
        if (!on_hull[p]) {
            std::vector<std::size_t> hit;
            for (std::size_t f = 0; f < faces.size(); ++f)
                if (active[f]) {
                    bool inside = true;
                    for (int i = 0; i < 3; ++i)
                        inside &=
                            orientation(points[faces[f][i]], points[faces[f][(i + 1) % 3]], points[p]) >= 0;
                    if (inside)
                        hit.push_back(f);
                }
            if (hit.empty() || hit.size() > 2)
                throw std::logic_error("Delaunay point location failed");
            std::vector<Triangle> old;
            for (auto f : hit) {
                old.push_back(faces[f]);
                remove(f);
            }
            for (auto t : old)
                for (int i = 0; i < 3; ++i)
                    if (orientation(points[t[i]], points[t[(i + 1) % 3]], points[p]))
                        add({t[i], t[(i + 1) % 3], p});
            legalize();
        }
    for (std::size_t f = 0; f < faces.size(); ++f)
        if (active[f])
            out.triangles.push_back(faces[f]);
    for (const auto &entry : adjacent)
        out.edges.push_back(entry.first);
    return out;
}
} // namespace cp::geometry::rational
