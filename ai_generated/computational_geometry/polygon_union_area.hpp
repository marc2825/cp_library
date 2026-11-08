#pragma once
#include "planar_embedding.hpp"
namespace cp::geometry::rational {
struct PolygonRegion {
    std::vector<Point> outer;
    std::vector<std::vector<Point>> holes;
};
// Simple outer/hole rings, holes inside their outer ring. Orientation is immaterial.
inline Scalar polygon_union_area(const std::vector<PolygonRegion> &regions, std::size_t max_pairs = 2000000,
                                 std::size_t max_candidates = 20000000) {
    std::vector<Segment> segments;
    for (const auto &region : regions) {
        auto add = [&](const std::vector<Point> &p) {
            if (p.size() < 3 || !signed_area2(p))
                return;
            for (std::size_t i = 0; i < p.size(); ++i)
                if (p[i] != p[(i + 1) % p.size()])
                    segments.push_back({p[i], p[(i + 1) % p.size()]});
        };
        add(region.outer);
        for (const auto &hole : region.holes)
            add(hole);
    }
    auto arrangement = segment_arrangement(segments, max_pairs, max_candidates);
    auto embedding = planar_embedding(arrangement);
    std::vector<Segment> split;
    for (auto e : arrangement.edges)
        split.push_back({arrangement.vertices[e.from], arrangement.vertices[e.to]});
    Scalar area;
    for (std::size_t f = 1; f < embedding.faces.size(); ++f) {
        const auto &face = embedding.faces[f];
        std::size_t start = face.boundaries[0][0];
        for (const auto &boundary : face.boundaries) {
            std::vector<Point> p;
            for (auto h : boundary)
                p.push_back(arrangement.vertices[embedding.half_edges[h].from]);
            if (signed_area2(p) > 0) {
                start = boundary[0];
                break;
            }
        }
        const auto &edge = embedding.half_edges[start];
        const auto q = embedding_detail::left_sample(
            {arrangement.vertices[edge.from], arrangement.vertices[edge.to]}, split);
        for (const auto &region : regions)
            if (contains(region.outer, q) == Location::inside) {
                bool hole = false;
                for (const auto &h : region.holes)
                    if (contains(h, q) != Location::outside) {
                        hole = true;
                        break;
                    }
                if (!hole) {
                    area += face.area;
                    break;
                }
            }
    }
    return area;
}
inline Scalar polygon_union_area(const std::vector<std::vector<Point>> &polygons,
                                 std::size_t max_pairs = 2000000, std::size_t max_candidates = 20000000) {
    std::vector<PolygonRegion> regions;
    for (const auto &p : polygons)
        regions.push_back({p, {}});
    return polygon_union_area(regions, max_pairs, max_candidates);
}
} // namespace cp::geometry::rational
