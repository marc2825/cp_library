#include "ordered_structure/all.hpp"
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#define CHECK(x) do { if (!(x)) { std::cerr << "line " << __LINE__ << ": " << #x << '\n'; std::abort(); } } while (false)
template <class Compare> void compare_state(const cp::CompressedOrderStatistics<int, Compare>& tree,
                                          const std::vector<int>& values, Compare compare) {
    CHECK(tree.size() == static_cast<long long>(values.size()) && tree.empty() == values.empty());
    for (long long k = -1; k <= static_cast<long long>(values.size()); ++k) {
        auto x = tree.kth(k);
        if (k < 0 || k == static_cast<long long>(values.size())) CHECK(!x);
        else CHECK(x && *x == values[static_cast<std::size_t>(k)]);
    }
    for (int x = -4; x <= 4; ++x) {
        long long before = 0, through = 0, count = 0;
        std::optional<int> previous, floor, next, ceiling;
        for (int v : values) {
            bool equal = !compare(v, x) && !compare(x, v);
            count += equal;
            before += compare(v, x); through += !compare(x, v);
            if (compare(v, x)) previous = v;
            if (!compare(x, v)) floor = v;
            if (!next && compare(x, v)) next = v;
            if (!ceiling && !compare(v, x)) ceiling = v;
        }
        CHECK(tree.count(x) == count && tree.order_of_key(x) == before && tree.count_less_equal(x) == through);
        CHECK(tree.predecessor(x) == previous && tree.predecessor(x, true) == floor);
        CHECK(tree.successor(x) == next && tree.successor(x, true) == ceiling);
        CHECK(tree.lower_bound(x) == ceiling && tree.upper_bound(x) == next);
        for (int y = -4; y <= 4; ++y) {
            long long expected = 0;
            if (compare(x, y)) for (int v : values) expected += !compare(v, x) && compare(v, y);
            CHECK(tree.count_range(x, y) == expected);
        }
    }
}

template <class Compare> void exhaustive(Compare compare) {
    for (int code = 0; code < 243; ++code) { // Each of five coordinates has multiplicity 0,1,2.
        cp::CompressedOrderStatistics<int, Compare> tree({-2, -1, 0, 1, 2, 0}, compare);
        std::vector<int> a; int number = code;
        for (int x = -2; x <= 2; ++x) {
            int count = number % 3; number /= 3;
            CHECK(tree.insert(x, count)); for (int i = 0; i < count; ++i) a.push_back(x);
        }
        std::sort(a.begin(), a.end(), compare); compare_state(tree, a, compare);
        for (int x = -3; x <= 3; ++x) {
            auto copy = tree; long long count = tree.count(x);
            CHECK(!copy.erase(x, count + 1)); compare_state(copy, a, compare);
            CHECK(copy.erase_all(x) == count);
            auto remaining = a; remaining.erase(std::remove(remaining.begin(), remaining.end(), x), remaining.end());
            compare_state(copy, remaining, compare);
            CHECK(tree.size() == static_cast<long long>(a.size()));
        }
    }
}

template <class Compare> void helper_history(std::mt19937& rng, Compare compare) {
    for (int round = 0; round < 80; ++round) {
        std::set<int, Compare> unique(compare);
        std::multiset<int, Compare> multiple(compare);
        std::map<int, int, Compare> mapping(compare);
        std::vector<int> a;
        for (int step = 0; step < 350; ++step) {
            int x = int(rng() % 9) - 4;
            if (rng() % 3) { multiple.insert(x); unique.insert(x); mapping.insert_or_assign(x, 10 * x); a.push_back(x); }
            else {
                auto found = std::find(a.begin(), a.end(), x);
                CHECK(cp::erase_one(multiple, x) == (found != a.end()));
                if (found != a.end()) a.erase(found);
                unique.erase(x); mapping.erase(x);
            }
            std::sort(a.begin(), a.end(), compare);
            for (bool inclusive : {false, true}) {
                auto check = [&](const auto& container, const std::vector<int>& ordered, auto key) {
                    std::optional<int> before, after;
                    for (int v : ordered) {
                        if (inclusive ? !compare(x, v) : compare(v, x)) before = v;
                        if (!after && (inclusive ? !compare(v, x) : compare(x, v))) after = v;
                    }
                    auto p = cp::predecessor(container, x, inclusive), s = cp::successor(container, x, inclusive);
                    CHECK((p == container.end()) == !before && (s == container.end()) == !after);
                    if (before) CHECK(key(*p) == *before);
                    if (after) CHECK(key(*s) == *after);
                };
                check(multiple, a, [](int v) { return v; });
                std::vector<int> keys(unique.begin(), unique.end());
                check(unique, keys, [](int v) { return v; });
                check(mapping, keys, [](const auto& v) { return v.first; });
            }
            CHECK(std::vector<int>(multiple.begin(), multiple.end()) == a);
            if (step % 40 == 0) {
                auto copy = multiple;
                for (int v : a) CHECK(cp::pop_first(copy) == std::optional<int>(v));
                CHECK(!cp::pop_first(copy) && !cp::pop_last(copy));
                copy = multiple;
                for (auto it = a.rbegin(); it != a.rend(); ++it) CHECK(cp::pop_last(copy) == std::optional<int>(*it));
            }
        }
    }
}

template <class Compare> void random_order_statistics(std::mt19937& rng, Compare compare) {
    for (int round = 0; round < 120; ++round) {
        cp::CompressedOrderStatistics<int, Compare> tree({-3, -2, -1, 0, 1, 2, 3}, compare);
        std::vector<int> a;
        for (int step = 0; step < 400; ++step) {
            int x = int(rng() % 11) - 5, count = int(rng() % 5), kind = int(rng() % 10);
            if (kind < 4) {
                bool valid = -3 <= x && x <= 3;
                CHECK(tree.insert(x, count) == valid);
                if (valid) for (int i = 0; i < count; ++i) a.push_back(x);
            } else if (kind < 7) {
                bool valid = -3 <= x && x <= 3 && std::count(a.begin(), a.end(), x) >= count;
                CHECK(tree.erase(x, count) == valid);
                if (valid) for (int i = 0; i < count; ++i) a.erase(std::find(a.begin(), a.end(), x));
            } else if (kind < 9) {
                CHECK(tree.erase_all(x) == std::count(a.begin(), a.end(), x));
                a.erase(std::remove(a.begin(), a.end(), x), a.end());
            } else { tree.clear(); a.clear(); }
            std::sort(a.begin(), a.end(), compare);
            compare_state(tree, a, compare);
        }
    }
}

struct Record { int key; explicit Record(int x) : key(x) {} }; // No default ctor or ==/<.
struct RecordCompare {
    bool reverse;
    explicit RecordCompare(bool r) : reverse(r) {}
    bool operator()(const Record& x, const Record& y) const { return reverse ? x.key > y.key : x.key < y.key; }
};
void boundaries() {
    cp::CoordinateCompression<int> empty;
    CHECK(empty.empty() && empty.lower_bound(3) == 0 && empty.upper_bound(3) == 0 && !empty.index(3));
    cp::CoordinateCompression<long long> coords({std::numeric_limits<long long>::max(), -7, -7, std::numeric_limits<long long>::min()});
    CHECK(coords.size() == 3 && *coords.index(-7) == 1 && !coords.index(0));
    CHECK(coords.lower_bound(0) == 2 && coords.upper_bound(-7) == 2);
    CHECK(coords.value(0) == std::numeric_limits<long long>::min());
    cp::CoordinateCompression<Record, RecordCompare> records({Record(4), Record(1), Record(4)}, RecordCompare(true));
    CHECK(records.size() == 2 && records.value(0).key == 4 && *records.index(Record(1)) == 1);
    cp::CompressedOrderStatistics<Record, RecordCompare> custom({Record(1), Record(4)}, RecordCompare(true));
    CHECK(custom.insert(Record(1), 2) && custom.insert(Record(4)));
    CHECK(custom.kth(0)->key == 4 && custom.kth(2)->key == 1 && custom.order_of_key(Record(2)) == 1);
    cp::CompressedOrderStatistics<std::string> strings({"a", "ab", "a", "z"});
    CHECK(strings.insert("a", 2) && strings.insert("z") && strings.kth(1) == std::optional<std::string>("a"));
    CHECK(strings.count_range("a", "b") == 2 && strings.lower_bound("b") == std::optional<std::string>("z"));
    cp::CompressedOrderStatistics<int> none;
    CHECK(none.empty() && !none.insert(1) && !none.erase(1) && none.count(1) == 0 && !none.kth(0));
    none.clear(); CHECK(none.universe_size() == 0 && none.count_range(0, 5) == 0);
    cp::CompressedOrderStatistics<int> huge({1, 2});
    constexpr long long maximum = std::numeric_limits<long long>::max();
    CHECK(huge.insert(1, maximum));
    CHECK(!huge.insert(2) && !huge.insert(1, -1) && !huge.erase(1, -1));
    CHECK(huge.insert(2, 0) && !huge.insert(3, 0) && huge.erase(2, 0));
    CHECK(huge.size() == maximum && huge.kth(maximum - 1) == std::optional<int>(1));
    CHECK(!huge.kth(maximum) && !huge.kth(std::numeric_limits<long long>::min()));
    CHECK(huge.erase(1, maximum - 1) && huge.insert(2, maximum - 1));
    CHECK(huge.order_of_key(2) == 1 && huge.count_less_equal(2) == maximum);
    CHECK(huge.kth(maximum - 1) == std::optional<int>(2) && huge.erase_all(2) == maximum - 1);
    CHECK(huge.size() == 1 && huge.erase(1) && huge.empty());
    std::map<std::string, int, std::less<>> words{{"a", 1}, {"c", 3}};
    std::string_view b = "b";
    CHECK(cp::predecessor(words, b)->second == 1 && cp::successor(words, b)->second == 3);
    cp::successor(words, b)->second = 4;
    const auto& constant = words;
    static_assert(std::is_same_v<decltype(cp::predecessor(constant, b)), decltype(words)::const_iterator>);
    CHECK(cp::predecessor(constant, std::string_view("c"), true)->second == 4);
    auto first = cp::pop_first(words), last = cp::pop_last(words);
    CHECK(first->first == "a" && last->second == 4 && words.empty());
    std::map<int, std::unique_ptr<int>> pointers;
    pointers.try_emplace(3, std::make_unique<int>(5));
    CHECK(*(cp::successor(pointers, 2)->second) == 5);
    CHECK(cp::erase_one(pointers, 3)); // Lookup and erasure need no mapped-type copying.
}

void stress() {
    constexpr int n = 200000;
    std::vector<int> universe(n); for (int i = 0; i < n; ++i) universe[i] = 2 * i;
    cp::CompressedOrderStatistics<int> tree(universe);
    for (int x : universe) CHECK(tree.insert(x));
    CHECK(tree.kth(n - 1) == std::optional<int>(2 * n - 2));
    for (int k = 0; k < n; ++k) CHECK(tree.kth(k) == std::optional<int>(2 * k));
    for (int i = 0; i < n; i += 2) CHECK(tree.erase(2 * i));
    CHECK(tree.size() == n / 2 && tree.order_of_key(n) == n / 4);
    tree.clear(); CHECK(tree.empty() && tree.universe_size() == n && tree.insert(0));
}

int main() {
    std::mt19937 rng(35017);
    boundaries(); exhaustive(std::less<int>{}); exhaustive(std::greater<int>{});
    helper_history(rng, std::less<int>{}); helper_history(rng, std::greater<int>{});
    random_order_statistics(rng, std::less<int>{}); random_order_statistics(rng, std::greater<int>{}); stress();
    std::cout << "ordered structures: 486 exhaustive frequency states, 400 seeded histories / 152000 mutations, ascending/descending order, standard containers, custom types, LLONG_MAX and 200000 coordinates passed\n";
}
