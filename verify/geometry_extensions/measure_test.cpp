#include "test_common.hpp"
using namespace htest;
int main() {
    std::mt19937_64 rng(20261028);
    for (int trial = 0; trial < 300; ++trial) {
        std::vector<e::Rectangle> rectangles;
        std::set<std::pair<int, int>> covered;
        for (int i = 0; i < 10; ++i) {
            int x1 = random_int(rng, -5, 5), x2 = random_int(rng, -5, 5), y1 = random_int(rng, -5, 5),
                y2 = random_int(rng, -5, 5);
            if (x1 > x2)
                std::swap(x1, x2);
            if (y1 > y2)
                std::swap(y1, y2);
            rectangles.push_back({x1, y1, x2, y2});
            for (int x = x1; x < x2; ++x)
                for (int y = y1; y < y2; ++y)
                    covered.emplace(x, y);
        }
        int perimeter = 0;
        for (auto [x, y] : covered)
            for (auto d : std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
                perimeter += !covered.count({x + d.first, y + d.second});
        auto result = e::rectangle_union_measure(rectangles);
        CHECK(result.area == BigInt(covered.size()));
        CHECK(result.perimeter == perimeter);
    }
    const auto lo = std::numeric_limits<std::int64_t>::min(), hi = std::numeric_limits<std::int64_t>::max();
    const BigInt width = BigInt(hi) - lo;
    auto extreme = e::rectangle_union_measure({{lo, lo, hi, hi}});
    CHECK(extreme.area == width * width && extreme.perimeter == 4 * width);
    for (int trial = 0; trial < 50; ++trial) {
        e::OriginRectangleUnion incremental;
        std::vector<e::Rectangle> rectangles;
        for (int i = 0; i < 100; ++i) {
            auto x = random_int(rng, 0, 30), y = random_int(rng, 0, 30);
            rectangles.push_back({0, 0, x, y});
            CHECK(incremental.insert(x, y) == e::rectangle_union_area(rectangles));
        }
    }
    throws<std::invalid_argument>([] { e::rectangle_union_measure({{2, 0, 1, 1}}); });
    throws<std::invalid_argument>([] { e::OriginRectangleUnion().insert(-1, 1); });
    const long double pi = std::acos(-1.L);
    const f::Circle unit{{0, 0}, 1};
    near(f::circle_intersection_area(unit, unit), pi);
    near(f::circle_intersection_area(unit, {{2, 0}, 1}), 0);
    near(f::circle_intersection_area(unit, {{1, 0}, 1}), 2 * pi / 3 - std::sqrt(3.L) / 2);
    near(f::circle_polygon_intersection_area(unit, {{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}), pi);
    near(f::circle_polygon_intersection_area(unit, {{0, 0}, {2, 0}, {2, 2}, {0, 2}}), pi / 4);
    near(f::circle_union_area({unit, unit, {{0, 0}, 0}, {{0, 0}, 0.5}}), pi);
    for (int trial = 0; trial < 100; ++trial) {
        f::Circle a{{static_cast<long double>(random_int(rng, -5, 5)),
                     static_cast<long double>(random_int(rng, -5, 5))},
                    static_cast<long double>(random_int(rng, 0, 5))},
            b{{static_cast<long double>(random_int(rng, -5, 5)),
               static_cast<long double>(random_int(rng, -5, 5))},
              static_cast<long double>(random_int(rng, 0, 5))};
        const auto intersection = f::circle_intersection_area(a, b);
        near(intersection, f::circle_intersection_area(b, a));
        near(f::circle_union_area({a, b}), pi * (a.radius * a.radius + b.radius * b.radius) - intersection);
        auto tangents = f::common_tangents(a, b);
        for (const auto &line : tangents.lines) {
            near(f::norm_squared(line.normal), 1);
            near(f::dot(line.normal, line.first), line.offset);
            near(f::dot(line.normal, line.second), line.offset);
            near(f::distance(line.first, a.center), a.radius);
            near(f::distance(line.second, b.center), b.radius);
        }
    }
    CHECK(f::common_tangents(unit, unit).infinite);
    CHECK(f::common_tangents({{0, 0}, 0}, {{1, 0}, 0}).lines.size() == 1);
    CHECK(f::common_tangents(unit, {{4, 0}, 1}).lines.size() == 4);
    CHECK(f::common_tangents(unit, {{2, 0}, 1}).lines.size() == 3);
    CHECK(f::tangents_from_point(unit, {1, 0}).lines.size() == 1);
    CHECK(f::tangents_from_point(unit, {0, 0}).lines.empty());
    for (int trial = 0; trial < 100; ++trial) {
        f::Point a{static_cast<long double>(random_int(rng, -5, 5)),
                   static_cast<long double>(random_int(rng, -5, 5))},
            b{static_cast<long double>(random_int(rng, -5, 5)),
              static_cast<long double>(random_int(rng, -5, 5))},
            c{static_cast<long double>(random_int(rng, -5, 5)),
              static_cast<long double>(random_int(rng, -5, 5))};
        auto centers = f::triangle_centers(a, b, c);
        if (!f::cross(a, b, c)) {
            CHECK(!centers);
            continue;
        }
        CHECK(centers);
        for (auto p : {a, b, c})
            near(f::distance(centers->circumcircle.center, p), centers->circumcircle.radius);
        for (auto line : {f::Line(a, b), f::Line(b, c), f::Line(c, a)})
            near(f::distance(line, centers->incircle.center), centers->incircle.radius);
        near(f::dot(centers->orthocenter - a, b - c), 0);
        auto centroid = f::polygon_centroid({a, b, c});
        CHECK(centroid);
        near(centroid->x, centers->centroid.x);
        near(centroid->y, centers->centroid.y);
    }
    CHECK(!r::polygon_centroid({{0, 0}, {1, 0}}));
    auto center = r::polygon_centroid({{0, 0}, {4, 0}, {4, 2}, {0, 2}});
    CHECK(center && *center == r::Point(2, 1));
    // Independent midpoint quadrature of vertical circle/polygon slices (a fixed rectangle).
    for (int trial = 0; trial < 8; ++trial) {
        const f::Circle circle{{0.3L, -0.2L}, 0.5L + trial / 4.L};
        long double integral = 0;
        const int samples = 40000;
        for (int i = 0; i < samples; ++i) {
            const auto x = -1.L + 3.L * (i + 0.5L) / samples;
            const auto h2 = circle.radius * circle.radius - (x - circle.center.x) * (x - circle.center.x);
            if (h2 > 0) {
                const auto h = std::sqrt(h2);
                integral +=
                    std::max(0.L, std::min(1.L, circle.center.y + h) - std::max(-1.L, circle.center.y - h)) *
                    3 / samples;
            }
        }
        near(f::circle_polygon_intersection_area(circle, {{-1, -1}, {2, -1}, {2, 1}, {-1, 1}}), integral,
             2e-6L);
    }
    // Independent vertical-slice quadrature for a three-circle union.
    std::vector<f::Circle> circles{{{-1, 0}, 1.5L}, {{1, 0}, 1.5L}, {{0, 1}, 1}};
    long double integral = 0;
    const int samples = 60000;
    for (int i = 0; i < samples; ++i) {
        const auto x = -2.5L + 5.L * (i + 0.5L) / samples;
        std::vector<std::pair<long double, long double>> ranges;
        for (auto c : circles) {
            auto h2 = c.radius * c.radius - (x - c.center.x) * (x - c.center.x);
            if (h2 > 0) {
                auto h = std::sqrt(h2);
                ranges.emplace_back(c.center.y - h, c.center.y + h);
            }
        }
        std::sort(ranges.begin(), ranges.end());
        long double end = -100, length = 0;
        for (auto p : ranges) {
            length += std::max(0.L, p.second - std::max(end, p.first));
            end = std::max(end, p.second);
        }
        integral += length * 5 / samples;
    }
    near(f::circle_union_area(circles), integral, 2e-6L);
    const auto base_union = f::circle_union_area(circles);
    for (auto &c : circles)
        c.center = c.center + f::Point{1e15L, -1e15L};
    near(f::circle_union_area(circles), base_union);
    near(f::circle_union_area({{{1e18L, -1e18L}, 1}}), std::acos(-1.L));
    std::cout << "measures: 300 rectangle grids, 5000 incremental inserts, circle integrals/tangents/centers "
                 "passed\n";
}
