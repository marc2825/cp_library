#include "static_query/all.hpp"
#include "test_values.hpp"
#include <algorithm>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using namespace static_test;
#define CHECK(x) do { if (!(x)) { std::cerr << "line " << __LINE__ << ": " << #x << '\n'; std::abort(); } } while (false)
using Table = cp::SparseTable<int, minimum, infinity>;
using AffineQueue = cp::SWAG<Affine, concatenate_affine, identity>;

template <class Compare> void check_array(const std::vector<int>& a, Compare compare) {
    int n = int(a.size());
    for (bool strict : {false, true}) {
        auto left = cp::previous_better_indices(a, strict, compare);
        auto right = cp::next_better_indices(a, strict, compare);
        cp::MonotonicStack<int, Compare> stack(strict, compare);
        for (int i = 0; i < n; ++i) {
            int l = -1, r = -1;
            auto better = [&](int j) { return strict ? compare(a[j], a[i]) : !compare(a[i], a[j]); };
            for (int j = i - 1; j >= 0; --j) if (better(j)) { l = j; break; }
            for (int j = i + 1; j < n; ++j) if (better(j)) { r = j; break; }
            CHECK(left[i] == l && right[i] == r && stack.push(a[i]) == l);
            CHECK(stack.top().index == i && stack.top().value == a[i]);
            CHECK(stack.processed_count() == i + 1);
            const auto& c = stack.candidates();
            for (int j = 1; j < int(c.size()); ++j) {
                CHECK(c[j - 1].index < c[j].index);
                CHECK(strict ? compare(c[j - 1].value, c[j].value) : !compare(c[j].value, c[j - 1].value));
            }
        }
        stack.clear(); CHECK(stack.empty() && stack.size() == 0 && stack.processed_count() == 0);
        CHECK(stack.push(4) == -1 && stack.top().index == 0);
    }
    for (int k = 1; k <= n + 1; ++k) {
        auto values = cp::sliding_window_minimum(a, k, compare);
        auto indices = cp::sliding_window_argmin(a, k, compare);
        std::vector<int> expected, positions;
        for (int l = 0; l + k <= n; ++l) {
            int best = l;
            for (int i = l + 1; i < l + k; ++i) if (compare(a[i], a[best])) best = i;
            expected.push_back(a[best]); positions.push_back(best);
        }
        CHECK(values == expected && indices == positions);
    }
}

void static_queries(const std::vector<int>& a) {
    int n = int(a.size()); Table table(a);
    CHECK(table.size() == n && table.empty() == a.empty());
    for (int l = 0; l <= n; ++l) {
        int value = infinity();
        for (int r = l; r <= n; ++r) {
            if (r > l) value = minimum(value, a[r - 1]);
            CHECK(table.prod(l, r) == value);
        }
    }
    for (int i = 0; i < n; ++i) CHECK(table.get(i) == a[i]);
    CHECK(table.all_prod() == (a.empty() ? infinity() : *std::min_element(a.begin(), a.end())));
}

void exhaustive() {
    for (int n = 0; n <= 8; ++n) {
        int cases = 1; for (int i = 0; i < n; ++i) cases *= 3;
        for (int mask = 0; mask < cases; ++mask) {
            std::vector<int> a(n); int value = mask;
            for (int& x : a) { x = value % 3 - 1; value /= 3; }
            static_queries(a); check_array(a, std::less<int>{}); check_array(a, std::greater<int>{});
        }
    }
}

void boundaries() {
    Table table; CHECK(table.prod(0, 0) == infinity() && table.all_prod() == infinity());
    table.build({4, 2, 3}); CHECK(table.prod(0, 3) == 2);
    table.build({7}); CHECK(table.prod(0, 1) == 7);
    table.build({}); CHECK(table.empty() && table.prod(0, 0) == infinity());
    cp::SparseTable<First, first, no_first> ordered({First(2), First(3), First(4), First(5), First(6)});
    for (int l = 0; l < 5; ++l) for (int r = l + 1; r <= 5; ++r) CHECK(ordered.prod(l, r).value == l + 2);
    CHECK(ordered.prod(3, 3).value == -1); // Idempotent and noncommutative.
    cp::SWAG<First, first, no_first> first_queue;
    first_queue.push(First(2)); first_queue.push(First(3));
    CHECK(first_queue.prod().value == 2); first_queue.pop(); CHECK(first_queue.prod().value == 3);
    first_queue.clear(); CHECK(first_queue.prod().value == -1);
    cp::MonotonicQueue<Item, ItemCompare> items(ItemCompare(false));
    items.push({2, 8}); items.push({2, 9});
    CHECK(items.best().payload == 8 && items.best_index() == 0);
    items.pop(); CHECK(items.best().payload == 9 && items.best_index() == 1);
    items.push(items.best()); items.pop(); CHECK(items.best().payload == 9 && items.best_index() == 2);
    items.clear(); items.push({1, 7}); CHECK(items.best_index() == 0 && items.size() == 1);
    cp::MonotonicStack<Item, ItemCompare> stack(true, ItemCompare(true));
    CHECK(stack.push({3, 8}) == -1 && stack.push({2, 9}) == 0);
    CHECK(stack.push(stack.top().value) == 0); // Aliased input must survive discarding the old top.
    std::vector<Item> data{{2, 0}, {1, 1}, {1, 2}, {3, 3}};
    auto argmin = cp::sliding_window_argmin(data, 3, ItemCompare(false));
    CHECK((argmin == std::vector<int>{1, 1}));
    auto minimums = cp::sliding_window_minimum(data, 3, ItemCompare(false));
    CHECK(minimums[0].payload == 1 && minimums[1].payload == 1);
}

void random_queues(std::mt19937& rng) {
    for (int round = 0; round < 240; ++round) {
        cp::MonotonicQueue<int> minimums; cp::MonotonicQueue<int, std::greater<int>> maximums;
        AffineQueue functions;
        cp::SWAG<std::string, concatenate, empty_string> strings;
        std::deque<int> values; std::deque<Affine> affine; std::deque<std::string> text;
        int left = 0, right = 0;
        for (int step = 0; step < 500; ++step) {
            int kind = int(rng() % 20);
            if (kind == 0) {
                minimums.clear(); maximums.clear(); functions.clear(); strings.clear();
                values.clear(); affine.clear(); text.clear(); left = right = 0;
            } else if (values.empty() || kind < 12) {
                int x = int(rng() % 21) - 10;
                Affine f{static_cast<long long>(rng() % mod), static_cast<long long>(rng() % mod)};
                std::string t(1, char('a' + rng() % 26));
                minimums.push(x); maximums.push(x); functions.push(f); strings.push(t);
                values.push_back(x); affine.push_back(f); text.push_back(t); ++right;
            } else {
                minimums.pop(); maximums.pop(); functions.pop(); strings.pop();
                values.pop_front(); affine.pop_front(); text.pop_front(); ++left;
            }
            CHECK(minimums.size() == int(values.size()) && maximums.empty() == values.empty());
            CHECK(functions.size() == int(values.size()) && strings.empty() == values.empty());
            long long expected = 17; std::string joined;
            for (const auto& f : affine) expected = evaluate(f, expected);
            for (const auto& t : text) joined += t;
            CHECK(evaluate(functions.prod(), 17) == expected && strings.prod() == joined);
            if (!values.empty()) {
                auto low = std::min_element(values.begin(), values.end()), high = std::max_element(values.begin(), values.end());
                CHECK(minimums.best() == *low && maximums.best() == *high);
                CHECK(minimums.best_index() == left + int(low - values.begin()));
                CHECK(maximums.best_index() == left + int(high - values.begin()));
                CHECK(functions.front().a == affine.front().a && functions.back().b == affine.back().b);
                CHECK(strings.front() == text.front() && strings.back() == text.back());
                for (const auto& entry : minimums.candidates()) CHECK(left <= entry.index && entry.index < right);
            }
        }
    }
}

void stress() {
    constexpr int n = 200000;
    std::vector<int> a(n); std::iota(a.begin(), a.end(), 0); Table table(a);
    CHECK(table.prod(14537, n) == 14537 && table.prod(n - 1, n) == n - 1);
    int comparisons = 0;
    cp::MonotonicStack<int, CountingLess> stack(true, CountingLess{&comparisons});
    for (int i = 0; i < n; ++i) stack.push(n - i);
    CHECK(comparisons < 2 * n);
    comparisons = 0;
    cp::MonotonicQueue<int, CountingLess> queue(CountingLess{&comparisons});
    for (int i = 0; i < n; ++i) queue.push(n - i);
    CHECK(comparisons < 2 * n && queue.size() == n && queue.candidates().size() == 1);
    for (int i = 0; i < n; ++i) { CHECK(queue.best() == 1); queue.pop(); }
    CHECK(queue.empty());
    auto mins = cp::sliding_window_minimum(a, 100001);
    CHECK(mins.size() == 100000 && mins.front() == 0 && mins.back() == 99999);
    operation_count = 0; cp::SWAG<long long, counted_sum, zero> sums;
    for (int i = 0; i < n; ++i) sums.push(1);
    for (int i = 0; i < n; ++i) { CHECK(sums.prod() == n - i); CHECK(sums.front() == 1); sums.pop(); }
    CHECK(sums.empty() && operation_count < 4 * n);
}

int main() {
    std::mt19937 rng(34017);
    boundaries(); exhaustive(); random_queues(rng); stress();
    std::cout << "static queries: 9841 exhaustive arrays, 240 FIFO histories / 120000 operations, ties, noncommutative order, custom types, 200000 elements and linear-operation bounds passed\n";
}
