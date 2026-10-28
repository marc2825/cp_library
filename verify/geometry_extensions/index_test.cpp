#include "test_common.hpp"
using namespace htest;
int main() {
    std::mt19937_64 rng(51);
    for (int trial = 0; trial < 200; ++trial) {
        std::vector<e::Point> points;
        for (int i = 0; i < 25; ++i)
            points.push_back({random_int(rng, -20, 20), random_int(rng, -20, 20)});
        const auto hull = robust::convex_hull(points);
        auto far = e::convex_farthest_neighbors(hull);
        for (std::size_t i = 0; i < hull.size(); ++i) {
            BigInt best;
            for (auto p : hull)
                best = std::max(best, robust::squared_distance(p, hull[i]));
            CHECK(far[i].squared_distance == best);
            CHECK(robust::squared_distance(hull[i], hull[far[i].index]) == best);
        }
        const e::TrianglePointCounter counter(points);
        for (int q = 0; q < 15; ++q) {
            e::Point a{random_int(rng, -20, 20), random_int(rng, -20, 20)},
                b{random_int(rng, -20, 20), random_int(rng, -20, 20)},
                c{random_int(rng, -20, 20), random_int(rng, -20, 20)};
            const auto shape = robust::convex_hull({a, b, c});
            for (bool boundary : {false, true}) {
                std::size_t expected = 0;
                for (auto p : points) {
                    const auto location =
                        cp::geometry::detail::convex_contains(shape, p, robust::IntegerKernel{});
                    expected += boundary ? location != Location::outside : location == Location::inside;
                }
                CHECK(counter.count(a, b, c, boundary) == expected);
            }
        }
    }
    for (std::size_t block_size : {1, 4, 16}) {
        e::DynamicConvexHull dynamic(block_size);
        std::multiset<e::Point> points;
        for (int step = 0; step < 2000; ++step) {
            e::Point p{random_int(rng, -10, 10), random_int(rng, -10, 10)};
            if (points.empty() || rng() % 3) {
                dynamic.insert(p);
                points.insert(p);
            } else {
                if (rng() % 2)
                    p = *std::next(points.begin(), rng() % points.size());
                auto it = points.find(p);
                CHECK(dynamic.erase(p) == (it != points.end()));
                if (it != points.end())
                    points.erase(it);
            }
            CHECK(dynamic.size() == points.size());
            e::Point direction{random_int(rng, -5, 5), random_int(rng, -5, 5)};
            auto best = dynamic.support(direction);
            CHECK(bool(best) == !points.empty());
            if (best) {
                const auto actual = BigInt(best->x) * direction.x + BigInt(best->y) * direction.y;
                for (auto q : points)
                    CHECK(BigInt(q.x) * direction.x + BigInt(q.y) * direction.y <= actual);
            }
            if (step % 25 == 0) {
                CHECK(dynamic.hull() ==
                      robust::convex_hull(std::vector<e::Point>(points.begin(), points.end())));
            }
        }
    }
    // Shrink successive blocks almost completely, then delete from the right end.
    // Exercises redistribution with a full neighbor and the last-block merge path.
    {
        e::DynamicConvexHull dynamic(32);
        std::set<e::Point> points;
        for (int x = 0; x < 512; ++x) {
            const e::Point p{x, (x * x) % 113};
            dynamic.insert(p);
            points.insert(p);
        }
        for (int x = 0; x < 512; ++x) {
            if (x % 32 == 0)
                continue;
            const e::Point p{x, (x * x) % 113};
            CHECK(dynamic.erase(p));
            points.erase(p);
            if (x % 7 == 0) {
                CHECK(dynamic.hull() ==
                      robust::convex_hull(std::vector<e::Point>(points.begin(), points.end())));
                for (e::Point direction : {e::Point{1, 2}, {-1, -2}, {0, 1}, {0, -1}}) {
                    const auto best = *dynamic.support(direction);
                    for (auto q : points)
                        CHECK(BigInt(best.x) * direction.x + BigInt(best.y) * direction.y >=
                              BigInt(q.x) * direction.x + BigInt(q.y) * direction.y);
                }
            }
        }
        while (!points.empty()) {
            CHECK(dynamic.erase(*points.rbegin()));
            points.erase(std::prev(points.end()));
            CHECK(dynamic.hull() == robust::convex_hull(std::vector<e::Point>(points.begin(), points.end())));
        }
        CHECK(dynamic.empty() && !dynamic.support({1, 1}));
    }
    for (int trial = 0; trial < 150; ++trial) {
        std::vector<r::Point> points;
        for (int i = 0; i < 10; ++i)
            points.push_back(random_point(rng, 4));
        const auto diagram = r::voronoi_diagram(points);
        const auto &d = diagram.delaunay;
        const auto hull = r::convex_hull(points);
        BigRational sum;
        for (auto t : d.triangles) {
            CHECK(r::orientation(d.vertices[t[0]], d.vertices[t[1]], d.vertices[t[2]]) > 0);
            sum += triangle_area(d.vertices[t[0]], d.vertices[t[1]], d.vertices[t[2]]);
            for (const auto &p : d.vertices)
                CHECK(robust::incircle(d.vertices[t[0]], d.vertices[t[1]], d.vertices[t[2]], p) <= 0);
        }
        CHECK(sum == cp::abs(r::signed_area2(hull)) / 2);
        if (hull.size() >= 3) {
            std::set<std::size_t> used;
            for (auto t : d.triangles)
                used.insert(t.begin(), t.end());
            CHECK(used.size() == d.vertices.size());
            auto embedding = r::planar_embedding(d.vertices, d.edges);
            CHECK(embedding.faces.size() == d.triangles.size() + 1);
        }
        for (const auto &edge : diagram.edges) {
            for (auto t : {BigRational(0), BigRational(1), BigRational(1, 2)}) {
                auto q = edge.origin + edge.direction * t;
                CHECK(r::squared_distance(q, d.vertices[edge.first_site]) ==
                      r::squared_distance(q, d.vertices[edge.second_site]));
                for (const auto &p : d.vertices)
                    CHECK(r::squared_distance(q, d.vertices[edge.first_site]) <= r::squared_distance(q, p));
            }
        }
        auto cells = r::voronoi_cells(diagram, {-6, -6}, {6, 6});
        BigRational covered;
        for (std::size_t i = 0; i < cells.size(); ++i) {
            covered += cp::abs(r::signed_area2(cells[i])) / 2;
            for (const auto &p : cells[i])
                for (const auto &q : d.vertices)
                    CHECK(r::squared_distance(p, d.vertices[i]) <= r::squared_distance(p, q));
        }
        CHECK(covered == 144);
    }
    auto cocircular = r::delaunay_triangulation({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    CHECK(cocircular.triangles.size() == 2 && cocircular.edges.size() == 5);
    auto collinear = r::voronoi_diagram({{0, 0}, {1, 0}, {2, 0}, {1, 0}});
    CHECK(collinear.edges.size() == 2 && collinear.delaunay.vertices.size() == 3);
    auto near_flat = r::delaunay_triangulation(
        {{0, 0}, {1, 0}, {2, BigRational(1, BigInt("1000000000000000000000000000000"))}, {0, 1}});
    CHECK(near_flat.triangles.size() == 2);
    const std::int64_t lo = std::numeric_limits<std::int64_t>::min(),
                       hi = std::numeric_limits<std::int64_t>::max();
    CHECK(robust::orientation(e::Point{lo, lo}, e::Point{hi, lo}, e::Point{lo, hi}) == 1);
    CHECK(robust::incircle(e::Point{lo, lo}, e::Point{hi, lo}, e::Point{hi, hi}, e::Point{lo, hi}) == 0);
    for (int exp : {-10000, -1000, -100, 0, 100, 1000, 10000}) {
        const auto value = std::ldexp(1.L, exp);
        f::Point a{0, 0}, b{value, value},
            c{value, std::nextafter(value, std::numeric_limits<long double>::infinity())};
        CHECK(robust::orientation(a, b, c) == 1);
        CHECK(robust::incircle(f::Point{0, 0}, f::Point{value, 0}, f::Point{value, value},
                               f::Point{0, value}) == 0);
    }
    throws<std::invalid_argument>([] { robust::dyadic(std::numeric_limits<long double>::infinity()); });
    std::vector<e::Point> parabola;
    for (int i = 0; i < 2000; ++i)
        parabola.push_back({i, 1LL * i * i});
    auto far = e::convex_farthest_neighbors(parabola);
    for (std::size_t i = 0; i < parabola.size(); i += 37) {
        BigInt maximum;
        for (auto p : parabola)
            maximum = std::max(maximum, robust::squared_distance(parabola[i], p));
        CHECK(far[i].squared_distance == maximum);
    }
    std::cout << "indexes: 200 all-farthest sets, 6000 triangle queries/dynamic updates, 150 exact "
                 "Delaunay/Voronoi, full-int64 and extreme-exponent predicates passed\n";
}
