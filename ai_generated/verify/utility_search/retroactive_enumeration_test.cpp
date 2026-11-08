#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include "basic_utility/retroactive_operations.hpp"
#include "basic_utility/special_integer_enumeration.hpp"
#include "range_query/monoid_catalog.hpp"

#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(#x) + " line " + std::to_string(__LINE__)); } while (false)
template<class E, class F> void throws(F f) {
    try { f(); } catch (const E&) { return; }
    throw std::runtime_error("expected exception");
}
struct Strings {
    using S = std::string;
    static S op(const S& a, const S& b) { return a + b; }
    static S e() { return {}; }
};
struct Sum {
    using S = std::int64_t;
    static S op(S a, S b) { return a + b; }
    static S e() { return 0; }
};
void timeline() {
    using Time = std::int64_t;
    const auto low = std::numeric_limits<Time>::min(), high = std::numeric_limits<Time>::max();
    cp::RetroactiveOperations<Strings> empty({});
    CHECK(empty.capacity() == 0 && empty.empty() && empty.all().empty());
    CHECK(empty.before(low).empty() && empty.through(high).empty());
    throws<std::out_of_range>([&] { empty.set(0, "a"); });
    throws<std::out_of_range>([&] { empty.get(0); });
    throws<std::out_of_range>([&] { empty.erase(0); });
    throws<std::invalid_argument>([&] { empty.fold(1, 0); });
    std::vector<Time> times{low, high};
    for (int i = -10; i <= 10; ++i) times.push_back(i * 3);
    auto duplicated = times; duplicated.insert(duplicated.end(), times.begin(), times.end());
    cp::RetroactiveOperations<Strings> t(duplicated);
    CHECK(t.capacity() == times.size());
    std::map<Time, std::string> model;
    std::mt19937 rng(22021);
    for (int step = 0; step < 3000; ++step) {
        const Time key = times[rng() % times.size()];
        if (rng() % 3) {
            const std::string s(rng() % 3, static_cast<char>('a' + rng() % 26));
            t.set(key, s); model[key] = s;
        } else CHECK(t.erase(key) == (model.erase(key) != 0));
        CHECK(t.size() == model.size() && t.empty() == model.empty());
        std::string all;
        for (const auto& [k, v] : model) { (void)k; all += v; }
        CHECK(t.all() == all);
        for (const auto time : times) {
            std::string before, through;
            for (const auto& [k, v] : model) { if (k < time) before += v; if (k <= time) through += v; }
            CHECK(t.before(time) == before && t.through(time) == through);
            CHECK(t.contains(time) == (model.count(time) != 0));
            const auto value = t.get(time);
            CHECK(value.has_value() == (model.count(time) != 0));
            if (value) CHECK(*value == model.at(time));
        }
        for (int trial = 0; trial < 10; ++trial) {
            Time l = static_cast<int>(rng() % 70) - 35, r = static_cast<int>(rng() % 70) - 35;
            if (l > r) std::swap(l, r);
            std::string want;
            for (const auto& [k, v] : model) if (l <= k && k < r) want += v;
            CHECK(t.fold(l, r) == want);
        }
    }
    CHECK(!t.contains(1));
    throws<std::out_of_range>([&] { t.set(1, "x"); });
    auto copy = t; copy.set(low, "copied"); CHECK(t.get(low) != copy.get(low));
    cp::RetroactiveOperations<cp::AffineFunctionMonoid<long long>> affine({0, 1, 2});
    affine.set(2, {3, 0}); affine.set(0, {2, 1}); affine.set(1, {0, 7});
    CHECK(affine.all()(99) == 21); // Noninvertible assignment in the past.
    affine.erase(1); CHECK(affine.all()(5) == 33);
    // libstdc++ debug checks lower_bound's partition precondition by a linear scan.
#ifdef _GLIBCXX_DEBUG
    constexpr int stress_size = 2000;
#else
    constexpr int stress_size = 200000;
#endif
    std::vector<Time> many(stress_size); std::iota(many.begin(), many.end(), Time(0));
    cp::RetroactiveOperations<Sum> stress(many);
    for (int i = stress_size - 1; i >= 0; --i) stress.set(i, i);
    CHECK(stress.all() == static_cast<Time>(stress_size - 1) * stress_size / 2);
    for (int i = 0; i < stress_size; i += 2) stress.erase(i);
    CHECK(stress.size() == stress_size / 2 && stress.all() == static_cast<Time>(stress_size / 2) * (stress_size / 2));
}

using U = std::uint64_t;
bool palindrome(U x) { auto s = std::to_string(x); auto rev = s; std::reverse(rev.begin(), rev.end()); return s == rev; }
bool allowed(U x, unsigned digits) {
    do { if (!(digits >> (x % 10) & 1)) return false; x /= 10; } while (x);
    return true;
}
void enumeration() {
    std::vector<U> actual, want;
    auto collect = [&](U x) { actual.push_back(x); return true; };
    CHECK(cp::enumerate_palindromic_integers(0, 1000000, collect));
    for (U x = 0; x < 1000000; ++x) if (palindrome(x)) want.push_back(x);
    CHECK(actual == want);
    for (unsigned mask = 0; mask < 1024; ++mask) {
        std::vector<int> digits;
        for (int d = 9; d >= 0; --d) if (mask >> d & 1) { digits.push_back(d); digits.push_back(d); }
        actual.clear(); want.clear();
        CHECK(cp::enumerate_digit_restricted_integers(0, 1000, digits, collect));
        for (U x = 0; x < 1000; ++x) if (allowed(x, mask)) want.push_back(x);
        CHECK(actual == want);
    }
    for (unsigned mask = 0; mask < 256; ++mask) {
        std::vector<U> factors;
        for (int f = 9; f >= 2; --f) if (mask >> (f - 2) & 1) { factors.push_back(f); factors.push_back(f); }
        actual.clear(); want.clear();
        CHECK(cp::enumerate_generated_products(500, factors, collect));
        std::vector<bool> reachable(500, false); reachable[1] = true;
        for (U x = 1; x < 500; ++x) {
            for (auto f : factors) if (x % f == 0 && reachable[x / f]) reachable[x] = true;
            if (reachable[x]) want.push_back(x);
        }
        CHECK(actual == want);
    }
    const auto max = std::numeric_limits<U>::max();
    std::mt19937_64 rng(22022);
    for (int trial = 0; trial < 600; ++trial) {
        const U first = trial < 300 ? rng() % 100000 : max - 2000 - rng() % 100000;
        const U last = first + rng() % 1000;
        actual.clear(); want.clear();
        CHECK(cp::enumerate_palindromic_integers(first, last, collect));
        for (auto x = first; x < last; ++x) if (palindrome(x)) want.push_back(x);
        CHECK(actual == want);
        std::vector<int> digits;
        const unsigned mask = trial % 2 ? 1023 : rng() % 1024;
        for (int d = 0; d <= 9; ++d) if (mask >> d & 1) digits.push_back(d);
        actual.clear(); want.clear();
        CHECK(cp::enumerate_digit_restricted_integers(first, last, digits, collect));
        for (auto x = first; x < last; ++x) if (allowed(x, mask)) want.push_back(x);
        CHECK(actual == want);
    }
    const U large_palindrome = 18446744066044764481ULL;
    actual.clear(); CHECK(cp::enumerate_palindromic_integers(large_palindrome, max, collect));
    CHECK(actual == std::vector<U>{large_palindrome});
    actual.clear(); CHECK(cp::enumerate_digit_restricted_integers(max - 1, max, {0,1,2,3,4,5,6,7,8,9}, collect));
    CHECK(actual == std::vector<U>{max - 1});
    actual.clear(); CHECK(cp::enumerate_generated_products(max, {2, 4, max - 1, max}, collect));
    want.clear(); for (int i = 0; i < 64; ++i) want.push_back(U(1) << i); want.push_back(max - 1);
    CHECK(actual == want);
    auto stop = [](U) { return false; };
    CHECK(!cp::enumerate_palindromic_integers(0, max, stop));
    CHECK(!cp::enumerate_digit_restricted_integers(0, max, {0}, stop));
    CHECK(!cp::enumerate_generated_products(max, {}, stop));
    int count = 0; auto third = [&](U) { return ++count < 3; };
    CHECK(!cp::enumerate_generated_products(max, {2, 3, 5}, third) && count == 3);
    count = 0; CHECK(!cp::enumerate_digit_restricted_integers(100, max, {1, 2}, third) && count == 3);
    count = 0; CHECK(!cp::enumerate_palindromic_integers(100, max, third) && count == 3);
    CHECK(cp::enumerate_palindromic_integers(max, max, stop));
    CHECK(cp::enumerate_digit_restricted_integers(0, max, {}, stop));
    CHECK(cp::enumerate_generated_products(1, {2}, stop));
    throws<std::invalid_argument>([&] { cp::enumerate_palindromic_integers(2, 1, stop); });
    throws<std::invalid_argument>([&] { cp::enumerate_digit_restricted_integers(2, 1, {}, stop); });
    for (int d : {-1, 10}) throws<std::invalid_argument>([&] { cp::enumerate_digit_restricted_integers(0, 0, {d}, stop); });
    for (U f : {0, 1}) throws<std::invalid_argument>([&] { cp::enumerate_generated_products(0, {f}, stop); });
    throws<std::runtime_error>([] { cp::enumerate_palindromic_integers(0, 1, [](U) -> bool { throw std::runtime_error("callback"); }); });
}
int main() {
    timeline(); enumeration();
    std::cout << "retroactive timeline: map replay / large timeline; integer enumeration: brute force / uint64 bounds passed\n";
}
