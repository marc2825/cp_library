#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>
#include "set_function/all.hpp"
#include "dp_optimization/subset_dp.hpp"

namespace {
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
template <class E, class Fn> void rejects(Fn fn) {
    bool caught = false; try { fn(); } catch (const E&) { caught = true; }
    require(caught, "expected exception");
}
int popcount(std::uint64_t x) { int count = 0; for (; x; x &= x - 1) ++count; return count; }
template <class T> std::vector<T> direct(const std::vector<T>& a, bool subset, bool inverse) {
    auto result = a;
    for (std::size_t s = 0; s < a.size(); ++s) {
        result[s] = T(0);
        for (std::size_t t = 0; t < a.size(); ++t) if (subset ? (s & t) == t : (s & t) == s) {
            if (inverse && (popcount(s ^ t) & 1)) result[s] -= a[t]; else result[s] += a[t];
        }
    }
    return result;
}
void enumerations() {
    std::size_t pairs = 0;
    for (int n = 0; n <= 10; ++n) for (std::uint64_t mask = 0; mask < (std::uint64_t(1) << n); ++mask) {
        std::vector<std::uint64_t> sub, super, nonempty, expected_sub, expected_super;
        for (std::uint64_t t = 0; t < (std::uint64_t(1) << n); ++t) {
            if ((t & mask) == t) expected_sub.push_back(t);
            if ((t & mask) == mask) expected_super.push_back(t);
        }
        std::reverse(expected_sub.begin(), expected_sub.end());
        cp::for_each_submask(mask, [&](auto t) { sub.push_back(t); });
        cp::for_each_submask(mask, [&](auto t) { nonempty.push_back(t); }, false);
        cp::for_each_supermask(mask, n, [&](auto t) { super.push_back(t); });
        require(sub == expected_sub && super == expected_super, "enumeration order and all masks");
        expected_sub.pop_back(); require(nonempty == expected_sub, "exclude empty subset");
        pairs += sub.size() + super.size();
    }
    const auto hi = std::uint64_t(1) << 63, full = ~std::uint64_t(0);
    std::vector<std::uint64_t> a, b;
    cp::for_each_submask(hi | 1, [&](auto x) { a.push_back(x); return false; });
    require(a == std::vector<std::uint64_t>({hi | 1, hi, 1, 0}), "highest bit and callback return ignored");
    cp::for_each_supermask(full ^ 3, 64, [&](auto x) { b.push_back(x); });
    require(b == std::vector<std::uint64_t>({full ^ 3, full ^ 2, full ^ 1, full}), "64-bit universe without wrapping");
    b.clear(); cp::for_each_supermask(full, 64, [&](auto x) { b.push_back(x); });
    require(b == std::vector<std::uint64_t>({full}), "full uint64 universe");
    b.clear(); cp::for_each_supermask(hi - 1, 63, [&](auto x) { b.push_back(x); });
    require(b == std::vector<std::uint64_t>({hi - 1}), "63-bit universe");
    int callbacks = 0;
    for (int bits : {-1, 65}) rejects<std::invalid_argument>([&] { cp::for_each_supermask(0, bits, [&](auto) { ++callbacks; }); });
    rejects<std::invalid_argument>([&] { cp::for_each_supermask(8, 3, [&](auto) { ++callbacks; }); });
    rejects<std::invalid_argument>([&] { cp::for_each_supermask(1, 0, [&](auto) { ++callbacks; }); });
    require(!callbacks, "invalid enumeration invokes no callbacks");
    std::cout << "2047 masks / " << pairs << " sub/super pairs, descending/ascending order, empty and 63/64-bit boundaries passed\n";
}
void check(const std::vector<long long>& a) {
    auto f = a; cp::subset_zeta_transform(f); require(f == direct(a, true, false), "subset zeta definition");
    cp::subset_mobius_transform(f); require(f == a, "subset zeta then inverse");
    f = a; cp::subset_mobius_transform(f); require(f == direct(a, true, true), "subset Mobius alternating definition");
    cp::subset_zeta_transform(f); require(f == a, "subset inverse then zeta");
    f = a; cp::superset_zeta_transform(f); require(f == direct(a, false, false), "superset zeta definition");
    cp::superset_mobius_transform(f); require(f == a, "superset roundtrip");
    f = a; cp::superset_mobius_transform(f); require(f == direct(a, false, true), "superset Mobius alternating definition");
    cp::superset_zeta_transform(f); require(f == a, "superset inverse then zeta");
    f = a; cp::sos_dp(f); require(f == direct(a, true, false), "SOS sum");
    auto lo = a, hi = a; cp::sos_dp(lo, [](auto x, auto y) { return std::min(x, y); }); cp::sos_dp(hi, [](auto x, auto y) { return std::max(x, y); });
    for (std::size_t s = 0; s < a.size(); ++s) {
        auto minimum = a[s], maximum = a[s];
        for (std::size_t t = 0; t < a.size(); ++t) if ((s & t) == t) { minimum = std::min(minimum, a[t]); maximum = std::max(maximum, a[t]); }
        require(lo[s] == minimum && hi[s] == maximum, "SOS min/max definitions");
    }
}
void exhaustive() {
    std::size_t cases = 0;
    for (int size : {1, 2, 4, 8}) {
        int count = 1; for (int i = 0; i < size; ++i) count *= 3;
        for (int code = 0; code < count; ++code) {
            std::vector<long long> a(size); int value = code;
            for (auto& x : a) { x = value % 3 - 1; value /= 3; }
            check(a); ++cases;
        }
    }
    std::mt19937_64 rng(319019);
    for (int trial = 0; trial < 180; ++trial) {
        std::vector<long long> a(std::size_t(1) << (rng() % 9)); for (auto& x : a) x = static_cast<int>(rng() % 2001) - 1000; check(a);
    }
    std::cout << cases << " ternary tables and 180 random tables: definitions of 4 transforms and SOS sum/min/max passed\n";
}
struct Mod12 {
    int value;
    explicit Mod12(int x = 0) : value((x % 12 + 12) % 12) {}
    Mod12& operator+=(Mod12 b) { value = (value + b.value) % 12; return *this; }
    Mod12& operator-=(Mod12 b) { value = (value - b.value + 12) % 12; return *this; }
    bool operator==(Mod12 b) const { return value == b.value; }
};
struct Item { int value, index; Item(int v, int i) : value(v), index(i) {} };
struct Count {
    static std::size_t calls;
    long long value;
    explicit Count(long long x = 0) : value(x) {}
    Count& operator+=(const Count& b) { value += b.value; ++calls; return *this; }
    Count& operator-=(const Count& b) { value -= b.value; ++calls; return *this; }
};
std::size_t Count::calls = 0;
void generic_types() {
    std::vector<Mod12> a; for (int i = 0; i < 32; ++i) a.emplace_back(i);
    const auto original = a, expected = direct(a, true, false);
    cp::subset_zeta_transform(a); require(a == expected, "composite modular type"); cp::subset_mobius_transform(a); require(a == original, "modular inverse");
    cp::superset_zeta_transform(a); require(a == direct(original, false, false), "modular upper sum"); cp::superset_mobius_transform(a); require(a == original, "modular upper inverse");
    std::vector<std::complex<long long>> complex{{1, 2}, {-3, 4}, {5, -6}, {7, 8}};
    const auto expected_complex = direct(complex, true, false); cp::subset_zeta_transform(complex); require(complex == expected_complex, "additive complex type");
    std::vector<Item> items{{9, 0}, {3, 1}, {3, 2}, {8, 3}};
    cp::sos_dp(items, [](Item x, Item y) { return x.value < y.value || (x.value == y.value && x.index < y.index) ? x : y; });
    require(items[3].value == 3 && items[3].index == 1, "SOS type without zero/default construction");
    std::vector<unsigned> bits{1, 2, 4, 8}; cp::sos_dp(bits, [](unsigned x, unsigned y) { return x | y; });
    require(bits == std::vector<unsigned>({1, 3, 5, 15}), "SOS bitwise or");
    for (int n = 0; n <= 20; ++n) {
        const auto size = std::size_t(1) << n, updates = n * (size / 2);
        std::vector<Count> f(size, Count(1));
        Count::calls = 0; cp::subset_zeta_transform(f); require(Count::calls == updates, "exact zeta operation count");
        for (std::size_t mask = 0; mask < size; ++mask) require(f[mask].value == (1LL << popcount(mask)), "large all-ones subset sums");
        Count::calls = 0; cp::subset_mobius_transform(f); require(Count::calls == updates, "exact inverse operation count");
        Count::calls = 0; cp::superset_zeta_transform(f); require(Count::calls == updates, "exact superset operation count");
        for (std::size_t mask = 0; mask < size; ++mask) require(f[mask].value == (1LL << (n - popcount(mask))), "large upper sums");
        Count::calls = 0; cp::superset_mobius_transform(f); require(Count::calls == updates, "exact upper inverse operation count");
        for (const auto& x : f) require(x.value == 1, "large inverse output");
    }
    std::cout << "mod12, complex, identity-free SOS and all 2^20 outputs with exact update counts passed\n";
}
void boundaries() {
    for (std::size_t size : {0U, 3U, 5U, 6U, 7U}) for (int op = 0; op < 6; ++op) {
        std::vector<int> f(size, 1); const auto saved = f;
        rejects<std::invalid_argument>([&] {
            switch (op) { case 0: cp::subset_zeta_transform(f); break; case 1: cp::subset_mobius_transform(f); break; case 2: cp::superset_zeta_transform(f); break; case 3: cp::superset_mobius_transform(f); break; case 4: cp::sos_dp(f); break; default: cp::sos_dp(f, [](int, int) -> int { throw std::runtime_error("must not call"); }); }
        });
        require(f == saved, "invalid shape keeps input unchanged");
    }
    for (int x = -128; x <= 127; ++x) for (int y = -128; y <= 127; ++y) for (int kind = 0; kind < 4; ++kind) {
        std::vector<signed char> f{static_cast<signed char>(x), static_cast<signed char>(y)};
        const bool subset = kind < 2, inverse = kind & 1;
        const int value = subset ? (inverse ? y - x : y + x) : (inverse ? x - y : x + y);
        bool overflow = false;
        try { if (kind == 0) cp::subset_zeta_transform(f); if (kind == 1) cp::subset_mobius_transform(f); if (kind == 2) cp::superset_zeta_transform(f); if (kind == 3) cp::superset_mobius_transform(f); }
        catch (const std::overflow_error&) { overflow = true; }
        require(overflow == (value < -128 || value > 127), "8-bit integer overflow oracle");
        if (!overflow) require(f[subset ? 1 : 0] == value, "8-bit result");
    }
    const auto hi = std::numeric_limits<long long>::max(), lo = std::numeric_limits<long long>::lowest();
    rejects<std::overflow_error>([&] { std::vector<long long> f{1, hi}; cp::subset_zeta_transform(f); });
    rejects<std::overflow_error>([&] { std::vector<long long> f{1, lo}; cp::subset_mobius_transform(f); });
    rejects<std::overflow_error>([] { std::vector<unsigned> f{1, 0}; cp::subset_mobius_transform(f); });
    rejects<std::overflow_error>([] { std::vector<unsigned long long> f{1, std::numeric_limits<unsigned long long>::max()}; cp::sos_dp(f); });
    std::vector<long long> partial{1, 1, hi, 0}; rejects<std::overflow_error>([&] { cp::subset_zeta_transform(partial); });
    require(partial == std::vector<long long>({1, 2, hi, hi}), "updates before overflow remain");
    std::vector<long long> singleton{lo}; cp::subset_mobius_transform(singleton); require(singleton[0] == lo, "0-dimensional table");
    std::cout << "30 invalid-shape rejections, 262144 int8 operations, signed/unsigned limits and partial mutation passed\n";
}
} // namespace
int main() { enumerations(); exhaustive(); generic_types(); boundaries(); }
