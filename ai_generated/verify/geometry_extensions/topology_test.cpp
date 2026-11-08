#include "test_common.hpp"
using namespace htest;
int main() {
    std::mt19937_64 rng(17);
    for (int trial = 0; trial < 150; ++trial) {
        std::vector<r::Segment> segments;
        for (int i = 0; i < 8; ++i)
            segments.push_back({random_point(rng), random_point(rng)});
        const auto crossings = r::segment_intersections(segments);
        std::size_t count = 0;
        for (std::size_t i = 0; i < segments.size(); ++i)
            for (std::size_t j = 0; j < i; ++j)
                count += !r::segment_intersection(segments[i], segments[j]).empty();
        CHECK(count == crossings.size());
        auto a = r::segment_arrangement(segments);
        std::vector<std::pair<std::size_t, std::size_t>> edges;
        for (const auto &edge : a.edges) {
            edges.emplace_back(edge.from, edge.to);
            for (auto i : edge.segments) {
                CHECK(r::on_segment(segments[i], a.vertices[edge.from]));
                CHECK(r::on_segment(segments[i], a.vertices[edge.to]));
            }
        }
        const auto embedded = r::planar_embedding(a.vertices, edges);
        std::vector<std::size_t> parent(a.vertices.size());
        std::iota(parent.begin(), parent.end(), 0);
        auto root = [&](std::size_t x) {
            while (x != parent[x])
                x = parent[x];
            return x;
        };
        for (auto [u, v] : edges)
            parent[root(u)] = root(v);
        std::size_t components = 0;
        for (std::size_t i = 0; i < parent.size(); ++i)
            components += root(i) == i;
        CHECK(a.vertices.size() + embedded.faces.size() == edges.size() + components + 1);
        for (std::size_t i = 1; i < embedded.faces.size(); ++i)
            CHECK(embedded.faces[i].area > 0);
    }
    const std::vector<r::Point> nested{{0, 0}, {6, 0}, {6, 6}, {0, 6}, {2, 2}, {4, 2},
                                       {4, 4}, {2, 4}, {3, 3}, {9, 9}, {10, 9}};
    const auto embedding = r::planar_embedding(
        nested, {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {9, 10}});
    CHECK(embedding.faces.size() == 3);
    std::multiset<BigRational> areas;
    for (std::size_t i = 1; i < embedding.faces.size(); ++i)
        areas.insert(embedding.faces[i].area);
    CHECK(areas == std::multiset<BigRational>({4, 32}));
    CHECK(embedding.dual_edges.back().first == 0 && embedding.dual_edges.back().second == 0);
    throws<std::invalid_argument>(
        [] { r::planar_embedding({{0, 0}, {2, 2}, {0, 2}, {2, 0}}, {{0, 1}, {2, 3}}); });
    throws<std::length_error>([] { r::segment_intersections({{{0, 0}, {2, 2}}, {{0, 2}, {2, 0}}}, 0); });
    for (int trial = 0; trial < 100; ++trial) {
        std::vector<r::Point> p;
        for (int i = 0; i < 6; ++i) {
            auto q = random_point(rng);
            p.push_back(q / BigRational(random_int(rng, 1, 3)));
        }
        const auto hull = r::convex_hull(p);
        std::vector<r::Point> grid;
        for (int x = -6; x <= 6; ++x)
            for (int y = -6; y <= 6; ++y)
                if (r::contains(hull, {x, y}) != Location::outside)
                    grid.emplace_back(x, y);
        CHECK(r::count_lattice_points(p) == BigInt(grid.size()));
        CHECK(r::integer_hull(p) == r::convex_hull(grid));
    }
    CHECK(r::count_lattice_points({{0, 0}, {1000000000, 0}, {1000000000, 1000000000}, {0, 1000000000}}) ==
          BigInt(1000000001) * 1000000001);
    throws<std::length_error>([] { r::integer_hull({{0, 0}, {1000000000, 1}}, 100); });
    for (int trial = 0; trial < 40; ++trial) {
        std::vector<e::Rectangle> rectangles;
        std::vector<std::vector<r::Point>> polygons;
        for (int i = 0; i < 4; ++i) {
            auto a = random_point(rng), b = random_point(rng);
            const auto x1 = std::min(a.x, b.x), x2 = std::max(a.x, b.x), y1 = std::min(a.y, b.y),
                       y2 = std::max(a.y, b.y);
            rectangles.push_back(
                {x1.numerator().to_integral<std::int64_t>(), y1.numerator().to_integral<std::int64_t>(),
                 x2.numerator().to_integral<std::int64_t>(), y2.numerator().to_integral<std::int64_t>()});
            polygons.push_back({{x1, y1}, {x2, y1}, {x2, y2}, {x1, y2}});
        }
        CHECK(r::polygon_union_area(polygons) == BigRational(e::rectangle_union_area(rectangles)));
    }
    std::vector<r::PolygonRegion> donut{
        {{{0, 0}, {6, 0}, {6, 6}, {0, 6}}, {{{2, 2}, {4, 2}, {4, 4}, {2, 4}}}}};
    CHECK(r::polygon_union_area(donut) == 32);
    donut.push_back({{{2, 2}, {4, 2}, {4, 4}, {2, 4}}, {}});
    CHECK(r::polygon_union_area(donut) == 36);
    for (int trial = 0; trial < 100; ++trial) {
        auto a = r::convex_hull({random_point(rng), random_point(rng), random_point(rng), random_point(rng)}),
             b = r::convex_hull({random_point(rng), random_point(rng), random_point(rng), random_point(rng)});
        auto intersection = r::convex_polygon_intersection(a, b);
        CHECK(intersection == r::convex_polygon_intersection(b, a));
        for (const auto &p : intersection)
            CHECK(r::contains(a, p) != Location::outside && r::contains(b, p) != Location::outside);
    }
    CHECK(
        r::convex_polygon_intersection({{0, 0}, {1, 0}, {1, 1}, {0, 1}}, {{1, 0}, {2, 0}, {2, 1}, {1, 1}}) ==
        std::vector<r::Point>({{1, 0}, {1, 1}}));
    std::vector<r::Point> concave{{0, 0}, {4, 0}, {4, 4}, {2, 2}, {0, 4}};
    for (int reverse = 0; reverse < 2; ++reverse) {
        const auto triangles = r::triangulate_polygon(concave);
        CHECK(triangles.size() == 3);
        BigRational sum;
        for (auto t : triangles) {
            CHECK(r::orientation(concave[t[0]], concave[t[1]], concave[t[2]]) > 0);
            sum += triangle_area(concave[t[0]], concave[t[1]], concave[t[2]]);
        }
        CHECK(sum == cp::abs(r::signed_area2(concave)) / 2);
        std::reverse(concave.begin(), concave.end());
    }
    throws<std::invalid_argument>([] { r::triangulate_polygon({{0, 0}, {2, 2}, {0, 2}, {2, 0}}); });
    std::cout << "topology: 150 arrangements/Euler checks, nesting/bridges/holes, 100 rational lattice "
                 "regions, unions/intersections/triangulations passed\n";
}
