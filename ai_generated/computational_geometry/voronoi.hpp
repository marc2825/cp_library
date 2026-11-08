#pragma once
#include "delaunay.hpp"
namespace cp::geometry::rational {
struct VoronoiEdge {
    enum class Kind { segment, ray, line };
    Kind kind;
    Point origin, direction;
    std::size_t first_site, second_site;
    // origin + t*direction: segment 0<=t<=1, ray t>=0, line any t.
};
struct VoronoiDiagram {
    DelaunayTriangulation delaunay;
    std::vector<VoronoiEdge> edges;
};
inline Point circumcenter(const Point &a, const Point &b, const Point &c) {
    const auto u = b - a, v = c - a;
    const auto d = 2 * cross(u, v);
    if (!d)
        throw std::domain_error("collinear circumcenter");
    const auto uu = dot(u, u), vv = dot(v, v);
    return a + Point{(v.y * uu - u.y * vv) / d, (u.x * vv - v.x * uu) / d};
}
inline VoronoiDiagram voronoi_diagram(std::vector<Point> points) {
    VoronoiDiagram out;
    out.delaunay = delaunay_triangulation(std::move(points));
    const auto &d = out.delaunay;
    using Edge = std::pair<std::size_t, std::size_t>;
    std::map<Edge, std::vector<std::pair<std::size_t, std::size_t>>> faces;
    std::vector<Point> centers;
    for (std::size_t f = 0; f < d.triangles.size(); ++f) {
        const auto t = d.triangles[f];
        centers.push_back(circumcenter(d.vertices[t[0]], d.vertices[t[1]], d.vertices[t[2]]));
        for (int j = 0; j < 3; ++j)
            faces[std::minmax(t[j], t[(j + 1) % 3])].emplace_back(f, j);
    }
    for (auto [u, v] : d.edges) {
        const auto &adjacent = faces[{u, v}];
        if (adjacent.empty()) {
            const auto delta = d.vertices[v] - d.vertices[u];
            out.edges.push_back(
                {VoronoiEdge::Kind::line, (d.vertices[u] + d.vertices[v]) / 2, {-delta.y, delta.x}, u, v});
        } else if (adjacent.size() == 1) {
            auto [f, j] = adjacent[0];
            const auto t = d.triangles[f];
            const auto delta = d.vertices[t[(j + 1) % 3]] - d.vertices[t[j]];
            out.edges.push_back({VoronoiEdge::Kind::ray, centers[f], {delta.y, -delta.x}, u, v});
        } else {
            const auto a = centers[adjacent[0].first], b = centers[adjacent[1].first];
            if (a != b)
                out.edges.push_back({VoronoiEdge::Kind::segment, a, b - a, u, v});
        }
    }
    return out;
}
// Finite cells in an explicit closed rectangle. One cell per sorted unique site.
inline std::vector<std::vector<Point>> voronoi_cells(const VoronoiDiagram &diagram, Point lower,
                                                     Point upper) {
    if (lower.x > upper.x || lower.y > upper.y)
        throw std::invalid_argument("reversed Voronoi clipping box");
    const auto &sites = diagram.delaunay.vertices;
    std::vector<std::vector<Point>> cells;
    std::vector<std::vector<std::size_t>> adjacent(sites.size());
    for (auto [a, b] : diagram.delaunay.edges) {
        adjacent[a].push_back(b);
        adjacent[b].push_back(a);
    }
    for (std::size_t i = 0; i < sites.size(); ++i) {
        auto cell = convex_hull({lower, {upper.x, lower.y}, upper, {lower.x, upper.y}});
        for (auto j : adjacent[i]) {
            const auto d = sites[j] - sites[i], middle = (sites[j] + sites[i]) / 2;
            cell = convex_cut(cell, middle, {-d.y, d.x});
        }
        cells.push_back(std::move(cell));
    }
    return cells;
}
} // namespace cp::geometry::rational
