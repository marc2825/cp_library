#include "test_common.hpp"
using namespace htest;
void verify_hull(const std::vector<s::exact::Point> &points) {
    auto h = s::exact::convex_hull(points);
    if (h.dimension != 3)
        return;
    std::map<std::pair<std::size_t, std::size_t>, int> edges;
    for (auto face : h.faces) {
        for (const auto &p : h.points)
            CHECK(s::exact::orientation(h.points[face[0]], h.points[face[1]], h.points[face[2]], p) <= 0);
        for (int j = 0; j < 3; ++j)
            ++edges[std::minmax(face[j], face[(j + 1) % 3])];
    }
    for (auto e : edges)
        CHECK(e.second == 2);
    CHECK(h.volume() > 0);
}
int main() {
    using P = s::exact::Point;
    std::vector<P> cube;
    for (int x : {0, 1})
        for (int y : {0, 1})
            for (int z : {0, 1})
                cube.push_back({x, y, z});
    auto hull = s::exact::convex_hull(cube);
    CHECK(hull.dimension == 3 && hull.volume() == 1);
    verify_hull(cube);
    CHECK(s::exact::convex_hull({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}).volume() == BigRational(1, 6));
    CHECK(s::exact::convex_hull({}).dimension == -1);
    CHECK(s::exact::convex_hull({{1, 1, 1}}).dimension == 0);
    CHECK(s::exact::convex_hull({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}}).dimension == 1);
    CHECK(s::exact::convex_hull(
              {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {BigRational(1, 2), BigRational(1, 2), 0}})
              .boundary.size() == 4);
    std::mt19937_64 rng(92);
    for (int trial = 0; trial < 80; ++trial) {
        std::vector<P> points;
        for (int i = 0; i < 12; ++i)
            points.push_back({random_int(rng, -2, 2), random_int(rng, -2, 2), random_int(rng, -2, 2)});
        verify_hull(points);
        const auto original = s::exact::convex_hull(points).volume();
        for (auto &p : points)
            p = p + P{100, -200, 500};
        CHECK(s::exact::convex_hull(points).volume() == original);
    }
    std::vector<P> lattice;
    for (int x = 0; x <= 2; ++x)
        for (int y = 0; y <= 2; ++y)
            for (int z = 0; z <= 2; ++z)
                lattice.push_back({x, y, z});
    verify_hull(lattice);
    CHECK(s::exact::convex_hull(lattice).volume() == 8);
    near(s::distance(s::Line({0, 0, 0}, {1, 0, 0}), s::Line({0, 0, 1}, {0, 1, 0})), 1);
    near(s::distance(s::Plane({0, 0, 1}, 2), s::Point{1, 2, 5}), 3);
    near(s::distance(s::Segment{{0, 0, 0}, {1, 0, 0}}, s::Point{2, 1, 0}), std::sqrt(2.L));
    auto line_plane = s::intersection(s::Line({0, 0, 0}, {1, 2, 3}), s::Plane({0, 0, 1}, 6));
    CHECK(line_plane.point);
    near(line_plane.point->x, 2);
    near(line_plane.point->y, 4);
    near(line_plane.point->z, 6);
    auto planes = s::intersection(s::Plane({1, 0, 0}, 2), s::Plane({0, 1, 0}, 3));
    CHECK(planes.line);
    near(planes.line->origin.x, 2);
    near(planes.line->origin.y, 3);
    CHECK(s::intersection(s::Plane({1, 0, 0}, 2), s::Plane({2, 0, 0}, 4)).relation ==
          cp::geometry::LineRelation::coincident);
    const s::Sphere sphere({0, 0, 0}, 2);
    auto intersections = s::intersection(s::Line({-5, 0, 0}, {1, 0, 0}), sphere);
    CHECK(intersections.size() == 2);
    near(intersections[0].x, -2);
    near(intersections[1].x, 2);
    auto circle = s::intersection(s::Plane({0, 0, 1}, 1), sphere);
    CHECK(circle);
    near(circle->radius, std::sqrt(3.L));
    auto two = s::intersection(sphere, s::Sphere({2, 0, 0}, 2));
    CHECK(two.circle);
    near(two.circle->center.x, 1);
    near(two.circle->radius, std::sqrt(3.L));
    CHECK(s::intersection(sphere, sphere).coincident);
    throws<std::invalid_argument>([] { s::Plane({0, 0, 0}, 1); });
    throws<std::invalid_argument>([] { s::Sphere({}, -1); });
    std::cout << "spatial: exact 3D hull manifold/volume, 80 translated sets, coplanar/collinear inputs, "
                 "line/plane/sphere intersections passed\n";
}
