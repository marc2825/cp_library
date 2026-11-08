#pragma once
#include "segment_arrangement.hpp"
#include <set>
namespace cp::geometry::rational {
namespace embedding_detail {
// A point strictly on the left, before the first other boundary along the normal ray.
inline Point left_sample(const Segment &base, const std::vector<Segment> &edges) {
    const auto middle = (base.a + base.b) / 2, v = base.b - base.a, normal = Point{-v.y, v.x};
    Scalar step = 1;
    auto consider = [&](const Scalar &t) {
        if (t > 0)
            step = std::min(step, t / 2);
    };
    for (const auto &edge : edges) {
        const auto w = edge.b - edge.a;
        const auto d = cross(normal, w);
        if (d) {
            const auto t = cross(edge.a - middle, w) / d, u = cross(edge.a - middle, normal) / d;
            if (u >= 0 && u <= 1)
                consider(t);
        } else if (!cross(edge.a - middle, normal)) {
            const auto norm = dot(normal, normal);
            consider(dot(edge.a - middle, normal) / norm);
            consider(dot(edge.b - middle, normal) / norm);
        }
    }
    return middle + normal * step;
}
} // namespace embedding_detail
struct PlanarEmbedding {
    struct HalfEdge {
        std::size_t from, to, twin, next, face;
    };
    struct Face {
        std::vector<std::vector<std::size_t>> boundaries;
        std::vector<std::size_t> isolated_vertices;
        Scalar area;
    };
    struct DualEdge {
        std::size_t first, second, primal_edge;
    };
    std::vector<HalfEdge> half_edges;
    std::vector<Face> faces;          // face 0 is the unique unbounded face; its area is not reported.
    std::vector<DualEdge> dual_edges; // bridge => a loop; parallel dual edges retained.
};
inline PlanarEmbedding planar_embedding(const std::vector<Point> &points,
                                        const std::vector<std::pair<std::size_t, std::size_t>> &edges,
                                        bool check_embedding = true) {
    if (std::set<Point>(points.begin(), points.end()).size() != points.size())
        throw std::invalid_argument("embedding vertices must be distinct");
    std::vector<Segment> segments;
    std::set<std::pair<std::size_t, std::size_t>> seen;
    for (auto [u, v] : edges) {
        if (u >= points.size() || v >= points.size() || u == v)
            throw std::invalid_argument("invalid embedding edge");
        if (!seen.insert(std::minmax(u, v)).second)
            throw std::invalid_argument("duplicate geometric edge");
        segments.push_back({points[u], points[v]});
    }
    if (check_embedding) {
        for (std::size_t i = 0; i < edges.size(); ++i) {
            for (std::size_t p = 0; p < points.size(); ++p)
                if (p != edges[i].first && p != edges[i].second && on_segment(segments[i], points[p]))
                    throw std::invalid_argument("split edge at vertex first");
            for (std::size_t j = 0; j < i; ++j) {
                const auto q = segment_intersection(segments[i], segments[j]);
                if (q.size() > 1)
                    throw std::invalid_argument("overlapping embedding edges");
                if (q.size() == 1) {
                    const auto &p = q[0];
                    if ((p != segments[i].a && p != segments[i].b) ||
                        (p != segments[j].a && p != segments[j].b))
                        throw std::invalid_argument("split crossings first");
                }
            }
        }
    }
    PlanarEmbedding out;
    out.faces.resize(1);
    std::vector<std::vector<std::size_t>> adjacency(points.size());
    for (auto [u, v] : edges) {
        const auto h = out.half_edges.size();
        out.half_edges.push_back({u, v, h + 1, 0, 0});
        out.half_edges.push_back({v, u, h, 0, 0});
        adjacency[u].push_back(h);
        adjacency[v].push_back(h + 1);
    }
    auto upper = [](const Point &p) { return p.y > 0 || (p.y == 0 && p.x > 0); };
    for (std::size_t v = 0; v < points.size(); ++v) {
        auto &list = adjacency[v];
        std::sort(list.begin(), list.end(), [&](auto a, auto b) {
            const auto x = points[out.half_edges[a].to] - points[v],
                       y = points[out.half_edges[b].to] - points[v];
            if (upper(x) != upper(y))
                return upper(x) > upper(y);
            return cross(x, y) > 0;
        });
        for (std::size_t i = 0; i < list.size(); ++i)
            out.half_edges[out.half_edges[list[i]].twin].next = list[(i + list.size() - 1) % list.size()];
    }
    struct Walk {
        std::vector<std::size_t> half;
        std::vector<Point> polygon;
        Scalar area2;
        std::size_t face = 0;
    };
    std::vector<Walk> walks;
    std::vector<bool> visited(out.half_edges.size());
    for (std::size_t start = 0; start < out.half_edges.size(); ++start)
        if (!visited[start]) {
            Walk walk;
            auto h = start;
            do {
                visited[h] = true;
                walk.half.push_back(h);
                walk.polygon.push_back(points[out.half_edges[h].from]);
                h = out.half_edges[h].next;
            } while (h != start);
            walk.area2 = signed_area2(walk.polygon);
            if (walk.area2 > 0) {
                walk.face = out.faces.size();
                out.faces.emplace_back();
            }
            walks.push_back(std::move(walk));
        }
    auto container = [&](const Point &p) {
        std::size_t id = 0;
        Scalar smallest;
        for (const auto &w : walks)
            if (w.area2 > 0 && contains(w.polygon, p) == Location::inside && (!id || w.area2 < smallest)) {
                id = w.face;
                smallest = w.area2;
            }
        return id;
    };
    for (auto &walk : walks) {
        if (walk.area2 <= 0) {
            const auto &edge = out.half_edges[walk.half[0]];
            walk.face =
                container(embedding_detail::left_sample({points[edge.from], points[edge.to]}, segments));
        }
        out.faces[walk.face].boundaries.push_back(walk.half);
        if (walk.face)
            out.faces[walk.face].area += walk.area2 / 2;
        for (auto h : walk.half)
            out.half_edges[h].face = walk.face;
    }
    for (std::size_t v = 0; v < points.size(); ++v)
        if (adjacency[v].empty())
            out.faces[container(points[v])].isolated_vertices.push_back(v);
    for (std::size_t i = 0; i < edges.size(); ++i)
        out.dual_edges.push_back({out.half_edges[2 * i].face, out.half_edges[2 * i + 1].face, i});
    return out;
}
inline PlanarEmbedding planar_embedding(const SegmentArrangement &arrangement) {
    std::vector<std::pair<std::size_t, std::size_t>> edges;
    for (const auto &e : arrangement.edges)
        edges.emplace_back(e.from, e.to);
    return planar_embedding(arrangement.vertices, edges, false);
}
} // namespace cp::geometry::rational
