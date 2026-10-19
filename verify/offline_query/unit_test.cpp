#include <algorithm>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <vector>
#include "offline_query/all.hpp"
#include "union_find/disjoint_set_union.hpp"

#define CHECK(x) do { if (!(x)) { std::cerr << "check failed at line " << __LINE__ << ": " #x "\n"; std::abort(); } } while (false)
using Point = cp::WeightedPoint<>;
using Rect = cp::Rectangle<>;

long long brute_sum(const std::vector<Point>& points, const Rect& q) {
    long long result = 0;
    for (auto p : points)
        if (q.left <= p.x && p.x < q.right && q.down <= p.y && p.y < q.up) result += p.weight;
    return result;
}

void test_mo() {
    int arrays = 0;
    for (int n = 0, combinations = 1; n <= 6; ++n, combinations *= 3) {
        for (int mask = 0; mask < combinations; ++mask) {
            ++arrays;
            std::vector<int> a(n);
            int v = mask;
            for (auto& x : a) { x = v % 3; v /= 3; }
            cp::Mo mo(n);
            CHECK(mo.size() == n);
            std::vector<std::pair<int, int>> queries;
            for (int l = 0; l <= n; ++l) for (int r = l; r <= n; ++r) {
                CHECK(mo.add_query(l, r) == int(queries.size()));
                queries.emplace_back(l, r);
            }
            CHECK(mo.query_count() == int(queries.size()));
            for (int width : {0, 1, n + 1}) {
                std::deque<int> active;
                std::vector<int> visits(queries.size());
                mo.run([&](int i) { CHECK(active.empty() || active.front() == i + 1); active.push_front(i); },
                       [&](int i) { CHECK(active.empty() || active.back() == i - 1); active.push_back(i); },
                       [&](int i) { CHECK(!active.empty() && active.front() == i); active.pop_front(); },
                       [&](int i) { CHECK(!active.empty() && active.back() == i); active.pop_back(); },
                       [&](int id) {
                           auto [l, r] = queries[id];
                           CHECK(int(active.size()) == r - l);
                           for (int i = l; i < r; ++i) CHECK(active[i - l] == i);
                           ++visits[id];
                       }, width);
                for (int count : visits) CHECK(count == 1);
            }
            std::vector<int> counts(3), answer(queries.size());
            int distinct = 0;
            mo.run([&](int i) { if (counts[a[i]]++ == 0) ++distinct; },
                   [&](int i) { CHECK(counts[a[i]] > 0); if (--counts[a[i]] == 0) --distinct; },
                   [&](int id) { answer[id] = distinct; });
            for (int id = 0; id < int(queries.size()); ++id) {
                auto [l, r] = queries[id];
                CHECK(answer[id] == int(std::set<int>(a.begin() + l, a.begin() + r).size()));
            }
        }
    }
    cp::Mo empty(4);
    int callbacks = 0;
    empty.run([&](int) { ++callbacks; }, [&](int) { ++callbacks; }, [&](int) { ++callbacks; });
    CHECK(callbacks == 0 && empty.query_order().empty());
    std::cout << "Mo: " << arrays << " exhaustive ternary arrays, all ranges, directional callbacks and block sizes passed\n";
}

void test_pbs(std::mt19937& rng) {
    for (int m = 0; m <= 80; ++m) {
        int state = -7, resets = 0, applied = 0, checked = 0;
        auto answer = cp::parallel_binary_search(m, m + 2,
            [&] { state = 0; ++resets; },
            [&](int i) { CHECK(i == state); ++state; ++applied; },
            [&](int q) { ++checked; return state >= q; });
        for (int q = 0; q <= m + 1; ++q) CHECK(answer[q] == q);
        int rounds = 0;
        for (int x = m + 1; x > 0; x >>= 1) ++rounds;
        CHECK(resets <= rounds && applied <= rounds * m && checked <= rounds * (m + 2));
    }
    int calls = 0;
    CHECK(cp::parallel_binary_search(100, 0, [&] { ++calls; }, [&](int) { ++calls; },
                                    [&](int) { ++calls; return true; }).empty());
    CHECK(calls == 0);
    for (int trial = 0; trial < 200; ++trial) {
        int n = 1 + rng() % 20, m = rng() % 55, q = 80;
        std::vector<std::pair<int, int>> edges(m), queries(q);
        for (auto& [u, v] : edges) { u = rng() % n; v = rng() % n; }
        for (auto& [u, v] : queries) { u = rng() % n; v = rng() % n; }
        std::vector<int> expected(q, m + 1);
        std::vector<std::vector<int>> adjacency(n);
        for (int t = 0; t <= m; ++t) {
            if (t) { auto [u, v] = edges[t - 1]; adjacency[u].push_back(v); adjacency[v].push_back(u); }
            for (int id = 0; id < q; ++id) if (expected[id] == m + 1) {
                auto [u, v] = queries[id];
                std::vector<bool> seen(n);
                std::vector<int> queue{u}; seen[u] = true;
                for (int p = 0; p < int(queue.size()); ++p)
                    for (int to : adjacency[queue[p]]) if (!seen[to]) { seen[to] = true; queue.push_back(to); }
                if (seen[v]) expected[id] = t;
            }
        }
        cp::DisjointSetUnion dsu(n);
        auto answer = cp::parallel_binary_search(m, q, [&] { dsu.reset(n); },
            [&](int i) { dsu.merge(edges[i].first, edges[i].second); },
            [&](int id) { auto [u, v] = queries[id]; return dsu.same(u, v); });
        CHECK(answer == expected);
    }
    std::cout << "PBS: every boundary for M=0..80, 200 graph histories against BFS, callback order/counts passed\n";
}

void test_rectangles(std::mt19937& rng) {
    for (int trial = 0; trial < 300; ++trial) {
        std::vector<Point> points;
        std::vector<Rect> rectangles;
        std::vector<long long> expected;
        cp::CDQRectangleSum<> cdq;
        for (int step = 0; step < 160; ++step) {
            auto coordinate = [&] { return static_cast<long long>(rng() % 11) - 5; };
            if (rng() % 2) {
                Point p{coordinate(), coordinate(), static_cast<long long>(rng() % 101) - 50};
                points.push_back(p); cdq.add_point(p.x, p.y, p.weight);
            } else {
                Rect r{coordinate(), coordinate(), coordinate(), coordinate()};
                if (rng() % 3) {
                    if (r.right < r.left) std::swap(r.left, r.right);
                    if (r.up < r.down) std::swap(r.down, r.up);
                }
                CHECK(cdq.add_query(r) == int(expected.size()));
                rectangles.push_back(r); expected.push_back(brute_sum(points, r));
            }
            if (step % 41 == 0) CHECK(cdq.solve() == expected);
        }
        const auto& const_cdq = cdq;
        CHECK(const_cdq.solve() == expected && const_cdq.solve() == expected);
        auto static_answer = cp::static_rectangle_sum(points, rectangles);
        for (int i = 0; i < int(rectangles.size()); ++i) CHECK(static_answer[i] == brute_sum(points, rectangles[i]));
        cdq.clear(); CHECK(cdq.query_count() == 0 && cdq.solve().empty());
        CHECK(cdq.add_query(0, 0, 1, 1) == 0 && cdq.solve() == std::vector<long long>{0});
    }
    // Every rectangle on a small grid, duplicates and all four boundary edges.
    std::vector<Point> grid;
    cp::CDQRectangleSum<> cdq;
    for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y)
        for (int w : {-3, 5}) { grid.push_back({x, y, w}); cdq.add_point(x, y, w); }
    std::vector<Rect> rectangles;
    std::vector<long long> expected;
    for (int l = -2; l <= 2; ++l) for (int d = -2; d <= 2; ++d)
        for (int r = -2; r <= 2; ++r) for (int u = -2; u <= 2; ++u) {
            Rect rect{l, d, r, u}; rectangles.push_back(rect);
            cdq.add_query(rect); expected.push_back(brute_sum(grid, rect));
        }
    CHECK(cp::static_rectangle_sum(grid, rectangles) == expected && cdq.solve() == expected);
    long long lo = std::numeric_limits<long long>::min(), hi = std::numeric_limits<long long>::max();
    grid = {{lo, lo, 7}, {hi, hi, 9}, {0, 0, -2}, {lo, hi, 3}};
    rectangles = {{lo, lo, hi, hi}, {lo, lo, 0, 0}, {0, 0, hi, hi}, {hi, hi, lo, lo}};
    expected.clear(); cdq.clear();
    for (auto p : grid) cdq.add_point(p.x, p.y, p.weight);
    for (auto r : rectangles) { cdq.add_query(r); expected.push_back(brute_sum(grid, r)); }
    CHECK(cp::static_rectangle_sum(grid, rectangles) == expected && cdq.solve() == expected);
    CHECK(cp::static_rectangle_sum(std::vector<Point>{}, rectangles) == std::vector<long long>(4, 0));
    CHECK(cp::static_rectangle_sum(grid, std::vector<Rect>{}).empty());
    std::cout << "Sweep/CDQ: 300 histories / 48000 operations, 625 grid rectangles, extreme coordinates, repeat/append/clear passed\n";
}

struct Coordinate {
    int value;
    explicit Coordinate(int x) : value(x) {}
    friend bool operator<(const Coordinate& a, const Coordinate& b) { return a.value < b.value; }
};
struct Weight {
    long long a, b;
    explicit Weight(int zero) : a(zero), b(zero) {}
    Weight(long long x, long long y) : a(x), b(y) {}
    Weight& operator+=(const Weight& w) { a += w.a; b += w.b; return *this; }
    friend Weight operator-(const Weight& x, const Weight& y) { return {x.a-y.a, x.b-y.b}; }
};
void test_types() {
    std::vector<cp::WeightedPoint<Coordinate, Weight>> p{{Coordinate(1), Coordinate(2), Weight(3, 5)}};
    std::vector<cp::Rectangle<Coordinate>> r{{Coordinate(0), Coordinate(0), Coordinate(2), Coordinate(3)}};
    auto answer = cp::static_rectangle_sum(p, r);
    CHECK(answer[0].a == 3 && answer[0].b == 5);
    cp::CDQRectangleSum<Coordinate, Weight> cdq;
    cdq.add_point(p[0].x, p[0].y, p[0].weight); cdq.add_query(r[0]);
    answer = cdq.solve(); CHECK(answer[0].a == 3 && answer[0].b == 5);
}

void test_large() {
#ifdef _GLIBCXX_DEBUG
    // lower_bound's partition validation is linear in libstdc++ debug mode.
    constexpr int n = 2000;
#else
    constexpr int n = 200000;
#endif
    cp::Mo mo(n);
    mo.add_query(n, n); mo.add_query(0, n); mo.add_query(n / 2 - 1, n / 2 + 1);
    long long sum = 0;
    mo.run([&](int i) { sum += i; }, [&](int i) { sum -= i; }, [&](int id) {
        CHECK(sum == (id == 0 ? 0 : id == 1 ? 1LL * n * (n-1) / 2 : n - 1));
    });
    int t = 0;
    auto answer = cp::parallel_binary_search(n, n + 2, [&] { t = 0; },
        [&](int i) { CHECK(t == i); ++t; }, [&](int q) { return t >= q; });
    for (int i = 0; i < n + 2; ++i) CHECK(answer[i] == i);
    std::vector<Point> points;
    cp::CDQRectangleSum<> cdq;
    for (int i = 0; i < n / 2; ++i) {
        cdq.add_query(-1, -1, n, n); // later additions must not affect this result
        cdq.add_point(i, i, 1);
    }
    auto sums = cdq.solve();
    for (int i = 0; i < n / 2; ++i) CHECK(sums[i] == i);
    for (int i = 0; i < n; ++i) points.push_back({i, -i, 1});
    CHECK(cp::static_rectangle_sum(points, std::vector<Rect>{{0, -n, n, 1}, {0, -n, n, 0}})
          == (std::vector<long long>{n, n-1}));
    std::cout << "large: " << n << " elements/states/operations and " << n * 5 / 2 << " CDQ events passed\n";
}

int main() {
    std::mt19937 rng(36001);
    test_mo(); test_pbs(rng); test_rectangles(rng); test_types(); test_large();
}
