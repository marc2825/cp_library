#include "range_query/all.hpp"
#include "data_structure/segment_tree.hpp"
#include "data_structure/binary_indexed_tree.hpp"
#include "test_values.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using namespace range_test;
#define CHECK(x) do { if (!(x)) { std::cerr << "line " << __LINE__ << ": " << #x << '\n'; std::abort(); } } while (false)
using Lazy = cp::LazySegtree<Sum, combine, range_test::empty, Affine, map_sum, compose, identity>;
using Dual = cp::DualSegtree<long long, Affine, range_test::apply, compose, identity>;
using Seg = cp::Segtree<long long, range_test::plus, zero>;
using StringSeg = cp::Segtree<std::string, concat, empty_string>;
using StringLazy = cp::LazySegtree<std::string, concat, empty_string, int, shift, add_shift, no_shift>;
bool small(long long x) { return x <= 9; }
bool short_sum(Sum x) { return x.length <= 3; }

void boundaries() {
    cp::FenwickTree<long long> f;
    cp::RangeAddRangeSumFenwickTree<long long> b;
    Seg s;
    Lazy l;
    Dual d;
    CHECK(f.size() == 0 && f.sum(0, 0) == 0 && f.lower_bound(-1) == 0 && f.lower_bound(1) == 0);
    CHECK(b.size() == 0 && b.sum(0) == 0);
    b.add(0, 0, 7);
    CHECK(s.size() == 0 && s.all_prod() == 0 && s.prod(0, 0) == 0);
    CHECK(s.max_right<small>(0) == 0 && s.min_left<small>(0) == 0);
    l.apply(0, 0, {0, 5});
    d.apply(0, 0, {0, 5});
    CHECK(d.size() == 0 && l.all_prod().length == 0 && l.prod(0, 0).value == 0);
    CHECK(l.max_right<short_sum>(0) == 0 && l.min_left<short_sum>(0) == 0);
    cp::FenwickTree<long long> zeroes(std::vector<long long>{0, 0, 4, 0, 2, 0});
    CHECK(zeroes.lower_bound(0) == 0 && zeroes.lower_bound(1) == 2);
    CHECK(zeroes.lower_bound(4) == 2 && zeroes.lower_bound(5) == 4 && zeroes.lower_bound(7) == 6);
    Lazy fresh(3); // e() has length 0; actual elements must be set to {value, 1}.
    CHECK(fresh.all_prod().length == 0);
    for (int i = 0; i < 3; ++i) fresh.set(i, {0, 1});
    fresh.apply(0, 3, {0, 7});
    CHECK(fresh.all_prod().value == 21 && fresh.all_prod().length == 3);
    CHECK(fresh.max_right<short_sum>(0) == 3 && fresh.min_left<short_sum>(3) == 0);
    Dual initial(3, 8);
    initial.apply(0, 3, {2, 1});
    initial.apply(1, 3, {0, 4});
    initial.apply(0, 2, {3, 2});
    CHECK(initial.get(0) == 53 && initial.get(1) == 14 && initial.get(2) == 4);
    initial.set(1, 9);
    initial.apply(1, {2, 3});
    CHECK(initial.get(1) == 21);
    // Legacy public storage semantics and function-pointer APIs remain available.
    BIT<long long> old(3);
    old.add(1, 4); old.set(2, 7);
    CHECK(old.n == 4 && old.val[2] == 4 && old.bit[2] == 4);
    CHECK(old.get(2) == 7 && old.sum(1, 3) == 11 && old.lower_bound(5) == 2);
    segtree<long long, range_test::plus, zero> legacy(std::vector<long long>{2, 3, 4, 5});
    CHECK(legacy.prod(1, 3) == 7 && legacy.max_right<small>(0) == 3 && legacy.min_left<small>(4) == 2);
    cp::FenwickTree<ModValue> mf(std::vector<ModValue>{-1, 4});
    mf.add(0, 2); mf.set(1, 7);
    CHECK(mf.sum(2).value == 8);
    cp::RangeAddRangeSumFenwickTree<ModValue> mb(std::vector<ModValue>{-1, 4});
    mb.add(0, 2, 2);
    CHECK(mb.sum(2).value == 7 && mb.get(0).value == 1);
}

void random_numeric(std::mt19937& rng) {
    for (int round = 0; round < 400; ++round) {
        int n = int(rng() % 66);
        std::vector<long long> a(n), c(n);
        std::vector<Sum> nodes(n);
        for (int i = 0; i < n; ++i) { a[i] = rng() % 20; c[i] = rng() % 20; nodes[i] = {c[i], 1}; }
        cp::FenwickTree<long long> fenwick(a);
        cp::RangeAddRangeSumFenwickTree<long long> bit(a);
        Seg seg(a);
        Lazy lazy(nodes);
        Dual dual(c);
        for (int step = 0; step < 300; ++step) {
            int l = int(rng() % (n + 1)), r = int(rng() % (n + 1));
            if (l > r) std::swap(l, r);
            int kind = int(rng() % 6);
            if (kind == 0 && n) {
                int p = int(rng() % n); long long x = int(rng() % 41) - 20;
                fenwick.add(p, x - a[p]); bit.set(p, x); seg.set(p, x); a[p] = x;
            } else if (kind == 1) {
                long long x = int(rng() % 21) - 10;
                bit.add(l, r, x);
                for (int i = l; i < r; ++i) { a[i] += x; fenwick.add(i, x); seg.set(i, a[i]); }
            } else if (kind == 2) {
                Affine f{static_cast<long long>(rng() % 4), static_cast<long long>(rng() % 17)};
                lazy.apply(l, r, f); dual.apply(l, r, f);
                for (int i = l; i < r; ++i) c[i] = range_test::apply(f, c[i]);
            } else if (kind == 3 && n) {
                int p = int(rng() % n); long long x = rng() % mod;
                lazy.set(p, {x, 1}); dual.set(p, x); c[p] = x;
            } else if (kind == 4 && n) {
                int p = int(rng() % n); Affine f{static_cast<long long>(rng() % 4), static_cast<long long>(rng() % 17)};
                lazy.apply(p, f); dual.apply(p, f); c[p] = range_test::apply(f, c[p]);
            }
            long long sum = std::accumulate(a.begin() + l, a.begin() + r, 0LL);
            CHECK(fenwick.sum(l, r) == sum && bit.sum(l, r) == sum && seg.prod(l, r) == sum);
            CHECK(fenwick.sum(r) == std::accumulate(a.begin(), a.begin() + r, 0LL));
            long long modular_sum = std::accumulate(c.begin() + l, c.begin() + r, 0LL) % mod;
            CHECK(lazy.prod(l, r).value == modular_sum && lazy.prod(l, r).length == r - l);
            CHECK(lazy.all_prod().value == std::accumulate(c.begin(), c.end(), 0LL) % mod);
            if (n) {
                int p = int(rng() % n);
                CHECK(fenwick.get(p) == a[p] && bit.get(p) == a[p] && seg.get(p) == a[p]);
                CHECK(lazy.get(p).value == c[p] && dual.get(p) == c[p]);
            }
            auto length_ok = [](Sum x) { return x.length <= 7; };
            CHECK(lazy.max_right(l, length_ok) == std::min(n, l + 7));
            CHECK(lazy.min_left(r, length_ok) == std::max(0, r - 7));
        }
        for (int i = 0; i < n; ++i) CHECK(dual.get(i) == c[i]);
    }
}

void random_search(std::mt19937& rng) {
    for (int round = 0; round < 200; ++round) {
        int n = int(rng() % 66);
        std::vector<long long> a(n);
        for (auto& x : a) x = rng() % 11;
        cp::FenwickTree<long long> bit(a);
        Seg seg(a);
        std::vector<Sum> nodes; for (auto x : a) nodes.push_back({x, 1});
        Lazy lazy(nodes);
        for (int step = 0; step < 100; ++step) {
            if (n && rng() % 2) {
                int p = int(rng() % n); a[p] = rng() % 11;
                bit.set(p, a[p]); seg.set(p, a[p]); lazy.set(p, {a[p], 1});
            } else if (n) {
                int l = int(rng() % (n + 1)), r = int(rng() % (n + 1));
                if (l > r) std::swap(l, r);
                long long x = rng() % 11;
                lazy.apply(l, r, {0, x});
                for (int i = l; i < r; ++i) { a[i] = x; bit.set(i, x); seg.set(i, x); }
            }
            long long w = int(rng() % 800) - 2;
            int expected = 0; long long sum = 0;
            if (w > 0) { while (expected < n && sum + a[expected] < w) sum += a[expected++]; }
            CHECK(bit.lower_bound(w) == expected);
            int l = int(rng() % (n + 1)), r = int(rng() % (n + 1));
            long long limit = rng() % 150;
            int right = l, left = r; sum = 0;
            while (right < n && sum + a[right] <= limit) sum += a[right++];
            sum = 0; while (left > 0 && sum + a[left - 1] <= limit) sum += a[--left];
            CHECK(seg.max_right(l, [=](long long x) { return x <= limit; }) == right);
            CHECK(seg.min_left(r, [=](long long x) { return x <= limit; }) == left);
            CHECK(lazy.max_right(l, [=](Sum x) { return x.value <= limit; }) == right);
            CHECK(lazy.min_left(r, [=](Sum x) { return x.value <= limit; }) == left);
        }
    }
}

void random_order(std::mt19937& rng) {
    for (int round = 0; round < 120; ++round) {
        int n = int(rng() % 33);
        std::vector<std::string> a(n, "a"); StringSeg seg(a); StringLazy lazy(a);
        for (int step = 0; step < 120; ++step) {
            int l = int(rng() % (n + 1)), r = int(rng() % (n + 1));
            if (l > r) std::swap(l, r);
            if (rng() % 2) {
                int f = int(rng() % 26); lazy.apply(l, r, f);
                for (int i = l; i < r; ++i) { a[i] = shift(f, a[i]); seg.set(i, a[i]); }
            } else if (n) {
                int p = int(rng() % n); a[p] = std::string(1, char('a' + rng() % 26));
                seg.set(p, a[p]); lazy.set(p, a[p]);
            }
            std::string expected; for (int i = l; i < r; ++i) expected += a[i];
            CHECK(seg.prod(l, r) == expected && lazy.prod(l, r) == expected);
            auto ok = [](std::string x) { return x.find('b') == std::string::npos; };
            int right = l, left = r;
            while (right < n && a[right] != "b") ++right;
            while (left > 0 && a[left - 1] != "b") --left;
            CHECK(seg.max_right(l, ok) == right && lazy.max_right(l, ok) == right);
            CHECK(seg.min_left(r, ok) == left && lazy.min_left(r, ok) == left);
        }
    }
}

void stress() {
    constexpr int n = 200000;
    std::vector<long long> a(n, 1); cp::FenwickTree<long long> bit(a);
    cp::RangeAddRangeSumFenwickTree<long long> ranges(a); Seg seg(a);
    Lazy lazy(std::vector<Sum>(n, {1, 1})); Dual dual(a);
    ranges.add(1, n - 1, 3); lazy.apply(1, n - 1, {2, 3}); dual.apply(1, n - 1, {2, 3});
    CHECK(bit.lower_bound(n) == n - 1 && bit.lower_bound(n + 1) == n);
    CHECK(ranges.sum(n) == 4LL * n - 6 && seg.prod(1, n - 1) == n - 2);
    CHECK(lazy.all_prod().value == 5LL * n - 8 && dual.get(n - 2) == 5 && dual.get(n - 1) == 1);
}

int main() {
    std::mt19937 rng(33017);
    boundaries(); random_numeric(rng); random_search(rng); random_order(rng); stress();
    std::cout << "range queries: boundaries, 720 seeded sequences / 154400 operations, noncommutative order, legacy, 200000 elements passed\n";
}
