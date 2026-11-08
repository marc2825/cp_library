#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>
#include "linear_recurrence/all.hpp"
#include "fps/bostan_mori.hpp"

namespace {
using Poly = std::vector<int>;
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
template <class E, class Fn> void rejects(Fn fn) {
    bool caught = false; try { fn(); } catch (const E&) { caught = true; }
    require(caught, "expected exception");
}
int norm(long long x, int p) { return static_cast<int>((x % p + p) % p); }
int scalar_power(int x, std::uint64_t n, int p) {
    int r = 1 % p;
    while (n) { if (n & 1) r = norm(1LL * r * x, p); x = norm(1LL * x * x, p); n >>= 1; }
    return r;
}
Poly direct(Poly a, const Poly& c, std::size_t n, int p) {
    for (auto& x : a) x = norm(x, p);
    const auto d = c.size();
    a.resize(std::max(a.size(), n));
    for (std::size_t i = d; i < n; ++i) for (std::size_t j = 0; j < d; ++j) a[i] = norm(a[i] + 1LL * norm(c[j], p) * a[i - 1 - j], p);
    return a;
}
bool fits(const Poly& s, const Poly& c, int p) {
    for (std::size_t i = c.size(); i < s.size(); ++i) {
        int predicted = 0;
        for (std::size_t j = 0; j < c.size(); ++j) predicted = norm(predicted + 1LL * c[j] * norm(s[i - 1 - j], p), p);
        if (predicted != norm(s[i], p)) return false;
    }
    return true;
}
// Each candidate order is an independent linear system; no BM state is used.
int minimum_order(const Poly& s, int p) {
    for (std::size_t d = 0; d <= s.size(); ++d) {
        std::vector<Poly> a(s.size() - d, Poly(d + 1));
        for (std::size_t i = d; i < s.size(); ++i) {
            for (std::size_t j = 0; j < d; ++j) a[i - d][j] = norm(s[i - 1 - j], p);
            a[i - d][d] = norm(s[i], p);
        }
        std::size_t rank = 0;
        for (std::size_t col = 0; col < d && rank < a.size(); ++col) {
            std::size_t pivot = rank;
            while (pivot < a.size() && !a[pivot][col]) ++pivot;
            if (pivot == a.size()) continue;
            std::swap(a[rank], a[pivot]);
            const auto scale = scalar_power(a[rank][col], p - 2, p);
            for (auto& x : a[rank]) x = norm(1LL * x * scale, p);
            for (std::size_t i = rank + 1; i < a.size(); ++i) {
                const auto factor = a[i][col];
                for (std::size_t j = col; j <= d; ++j) a[i][j] = norm(a[i][j] - 1LL * factor * a[rank][j], p);
            }
            ++rank;
        }
        bool consistent = true;
        for (const auto& row : a) if (std::all_of(row.begin(), row.end() - 1, [](int x) { return x == 0; }) && row.back()) consistent = false;
        if (consistent) return static_cast<int>(d);
    }
    throw std::runtime_error("unreachable order");
}
template <int P> std::size_t exhaustive_bm(int maximum) {
    std::size_t cases = 0;
    for (int n = 0, count = 1; n <= maximum; ++n, count *= P) for (int code = 0; code < count; ++code) {
        Poly s(n); int value = code;
        for (auto& x : s) { x = value % P; value /= P; }
        const auto saved = s, c = cp::berlekamp_massey<P>(s);
        require(fits(s, c, P) && static_cast<int>(c.size()) == minimum_order(s, P), "BM minimal order by Gaussian elimination");
        if (c.empty()) require(std::all_of(s.begin(), s.end(), [](int x) { return !x; }), "zero-order sequence");
        require(cp::linear_recurrence_from_prefix<P>(s, s.size() ? s.size() - 1 : 0) == (s.empty() ? 0 : s.back()), "observed prefix lookup");
        if (!c.empty()) {
            const auto extended = direct(Poly(s.begin(), s.begin() + c.size()), c, s.size() + 4, P);
            require(std::equal(s.begin(), s.end(), extended.begin()), "BM selected extension fits whole prefix");
            require(cp::linear_recurrence_from_prefix<P>(s, s.size() + 3) == extended[s.size() + 3], "BM and Kitamasa extension");
        }
        require(saved == s, "BM input immutable"); ++cases;
    }
    return cases;
}
template <int P> void random_ring(std::mt19937_64& rng) {
    for (int trial = 0; trial < 100; ++trial) {
        const std::size_t d = 1 + rng() % 10, n = rng() % 150;
        Poly initial(d), c(d);
        for (auto& x : initial) x = static_cast<int>(static_cast<long long>(rng() % 4000000001ULL) - 2000000000);
        for (auto& x : c) x = static_cast<int>(static_cast<long long>(rng() % 4000000001ULL) - 2000000000);
        const auto saved_initial = initial, saved_c = c;
        const auto sequence = direct(initial, c, std::max(d * 2, n + 1), P);
        require(cp::kitamasa<P>(initial, c, n) == sequence[n], "Kitamasa vs direct recurrence");
        require(cp::linear_recurrence_matrix<P>(initial, c, n) == sequence[n], "matrix vs direct recurrence");
        const auto weights = cp::kitamasa_coefficients<P>(c, n);
        require(weights.size() == d, "weight vector keeps order");
        for (std::size_t i = 0; i < d; ++i) {
            Poly basis(d); basis[i] = 1 % P;
            require(weights[i] == direct(basis, c, n + 1, P)[n], "Kitamasa weights vs independent unit initial states");
        }
        if constexpr (P == 17 || P == 998244353) {
            const auto inferred = cp::berlekamp_massey<P>(Poly(sequence.begin(), sequence.begin() + 2 * d));
            require(inferred.size() <= d && fits(sequence, inferred, P), "2d observations recover recurrence extension");
            if constexpr (P == 998244353) require(cp::linear_recurrence_nth(initial, c, n) == sequence[n], "existing Bostan-Mori agreement");
        }
        require(initial == saved_initial && c == saved_c, "recurrence inputs immutable");
    }
}
void matrices(std::mt19937_64& rng) {
    for (int trial = 0; trial < 160; ++trial) {
        const std::size_t d = rng() % 6, n = rng() % 22;
        std::vector<Poly> a(d, Poly(d)); Poly s(d);
        for (auto& row : a) for (auto& x : row) x = static_cast<int>(rng() % 25) - 12;
        for (auto& x : s) x = static_cast<int>(rng() % 25) - 12;
        auto expected = s; for (auto& x : expected) x = norm(x, 12);
        for (std::size_t k = 0; k < n; ++k) {
            Poly next(d);
            for (std::size_t i = 0; i < d; ++i) for (std::size_t j = 0; j < d; ++j) next[i] = norm(next[i] + 1LL * a[i][j] * expected[j], 12);
            expected = next;
        }
        require(cp::matrix_power_apply<12>(a, s, n) == expected, "arbitrary state transition vs repeated stepping");
        const auto power = cp::matrix_power_mod<12>(a, n);
        for (std::size_t j = 0; j < d; ++j) {
            Poly basis(d); basis[j] = 1;
            auto column = cp::matrix_power_apply<12>(a, basis, n);
            for (std::size_t i = 0; i < d; ++i) require(power[i][j] == column[i], "matrix power columns");
        }
    }
    std::cout << "160 arbitrary matrices including 0x0, composite modulus and column-vector orientation passed\n";
}
void boundaries() {
    constexpr auto big = std::numeric_limits<std::uint64_t>::max();
    for (auto n : {std::uint64_t(0), std::uint64_t(1), std::uint64_t(1) << 63, big}) {
        const auto geometric = norm(7LL * scalar_power(3, n, 998244353), 998244353);
        require(cp::kitamasa({7}, {3}, n) == geometric, "uint64 Kitamasa");
        require(cp::linear_recurrence_matrix({7}, {3}, n) == geometric, "uint64 matrix recurrence");
        require(cp::linear_recurrence_from_prefix({7, 21, 63, 189}, n) == geometric, "uint64 inferred recurrence");
        require(cp::linear_recurrence_nth({7}, {3}, n) == geometric, "uint64 shared Bostan-Mori");
        const auto state = cp::matrix_power_apply<12>({{1, 1}, {0, 1}}, {4, 1}, n);
        require(state == Poly({static_cast<int>((n % 12 + 4) % 12), 1}), "affine state with uint64 steps");
        require(cp::matrix_power_mod<1>({{1, 2}, {3, 4}}, n) == std::vector<Poly>(2, Poly(2)), "zero ring identity");
    }
    require(cp::berlekamp_massey(Poly{}) == Poly{} && cp::berlekamp_massey(Poly(100)) == Poly{}, "empty/zero BM");
    require(cp::linear_recurrence_from_prefix({}, big) == 0, "empty inferred zero sequence");
    require(cp::berlekamp_massey(Poly{1, 0, 0, 0}) == Poly({0}), "trailing zero preserves order");
    require(cp::berlekamp_massey(Poly{0, 0, 1}) .size() == 3, "late discrepancy increases order beyond half");
    require(cp::kitamasa_coefficients<12>({0, 0, 0}, big) == Poly(3), "vanishing recurrence");
    require(cp::kitamasa<2000000000>({std::numeric_limits<int>::min()}, {std::numeric_limits<int>::max()}, big) >= 0, "largest composite modulus and int extremes");
    require(cp::berlekamp_massey<1000000007>({1, 3, 9, 27}) == Poly({3}), "non-NTT prime inference");
    rejects<std::invalid_argument>([] { cp::kitamasa({}, {}, 0); });
    rejects<std::invalid_argument>([] { cp::kitamasa({1}, {1, 2}, 0); });
    rejects<std::invalid_argument>([] { cp::kitamasa_coefficients({}, 0); });
    rejects<std::invalid_argument>([] { cp::linear_recurrence_matrix({}, {}, 0); });
    rejects<std::invalid_argument>([] { cp::matrix_power_mod({{1, 2}}, 0); });
    rejects<std::invalid_argument>([] { cp::matrix_power_mod({{1, 2}, {3}}, 0); });
    rejects<std::invalid_argument>([] { cp::matrix_power_apply({{1}}, {}, 0); });
    rejects<std::domain_error>([] { cp::bostan_mori({1}, {0, 1}, 0); });
    rejects<std::length_error>([] { cp::linear_recurrence_nth<17, 3>(Poly(8), Poly(8), 8); });
    require(cp::linear_recurrence_nth<17, 3>(Poly(8, -1), Poly(8), 0) == 16, "existing initial shortcut compatibility");
    require(cp::matrix_power_mod<17>({{1, 1}, {1, 0}}, 10)[0][1] == 4, "matrix recurrence sample");
    std::cout << "zero/order-one/late discrepancies, modulus1, non-NTT prime, uint64/int extremes, invalid dimensions and matrix recurrence sample passed\n";
}
void large() {
    constexpr int p = 998244353;
    constexpr std::uint64_t n = std::uint64_t(1) << 63;
    Poly initial(192), c(192); initial[0] = 7; c.back() = 1;
    require(cp::kitamasa(initial, c, n) == (n % 192 == 0 ? 7 : 0), "large Kitamasa periodic recurrence");
    Poly samples(2048);
    for (std::size_t i = 0; i < samples.size(); i += 1024) samples[i] = 1;
    const auto found = cp::berlekamp_massey(samples);
    require(found.size() == 1024 && found.back() == 1 && std::count(found.begin(), found.end(), 0) == 1023, "large BM known exact order");
    const auto state = cp::matrix_power_apply(std::vector<Poly>(48, Poly(48, 1)), Poly(48, 1), n);
    require(state == Poly(48, scalar_power(48, n, p)), "large dense matrix all entries");
    initial.assign(16384, 0); c.assign(16384, 0); initial[0] = 7; c.back() = 1;
    require(cp::linear_recurrence_nth(initial, c, n) == 7, "large Bostan-Mori NTT and huge index");
    require(cp::bostan_mori({3, -1, 8, 9}, {2}, 3) == norm(9LL * scalar_power(2, p - 2, p), p), "improper rational coefficient");
    std::cout << "order192 Kitamasa, 2048-term/order1024 BM, 48x48 dense state, order16384 Bostan-Mori at 2^63 passed\n";
}
} // namespace
int main() {
    std::mt19937_64 rng(318018);
    boundaries();
    const auto count = exhaustive_bm<2>(10) + exhaustive_bm<3>(7) + exhaustive_bm<5>(5);
    std::cout << count << " finite prefixes: BM minimum degree vs independent Gaussian systems passed\n";
    random_ring<1>(rng); random_ring<2>(rng); random_ring<12>(rng); random_ring<17>(rng);
    random_ring<998244353>(rng); random_ring<2000000000>(rng);
    std::cout << "600 random recurrences and every Kitamasa weight vs direct unit-state sequences passed\n";
    matrices(rng); large();
}
