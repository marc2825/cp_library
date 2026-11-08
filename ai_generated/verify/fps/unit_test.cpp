#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>
#include "fps/all.hpp"

namespace {
using Poly = std::vector<int>;
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
template <class E, class Fn> void rejects(Fn fn) {
    bool caught = false;
    try { fn(); } catch (const E&) { caught = true; }
    require(caught, "expected exception");
}
int norm(long long x, int p) { return static_cast<int>((x % p + p) % p); }
int scalar_power(int a, std::uint64_t k, int p) {
    int r = 1;
    while (k) { if (k & 1) r = static_cast<int>(1LL * r * a % p); a = static_cast<int>(1LL * a * a % p); k >>= 1; }
    return r;
}
Poly product(const Poly& a, const Poly& b, int p, std::size_t n) {
    Poly c(n);
    for (std::size_t i = 0; i < a.size() && i < n; ++i) for (std::size_t j = 0; j < b.size() && i + j < n; ++j)
        c[i + j] = norm(c[i + j] + 1LL * norm(a[i], p) * norm(b[j], p), p);
    return c;
}
int coefficient(const Poly& a, std::size_t i, int p) { return i < a.size() ? norm(a[i], p) : 0; }
Poly inverse(const Poly& a, int p, std::size_t n) {
    Poly b(n); if (!n) return b;
    const int scale = scalar_power(norm(a[0], p), p - 2, p); b[0] = scale;
    for (std::size_t i = 1; i < n; ++i) {
        int sum = 0;
        for (std::size_t j = 1; j <= i; ++j) sum = norm(sum + 1LL * coefficient(a, j, p) * b[i - j], p);
        b[i] = norm(-1LL * sum * scale, p);
    }
    return b;
}
Poly exponential(const Poly& a, int p, std::size_t n) {
    Poly b(n); if (n) b[0] = 1;
    for (std::size_t i = 1; i < n; ++i) {
        int sum = 0;
        for (std::size_t j = 1; j <= i; ++j) sum = norm(sum + (1LL * j * coefficient(a, j, p) % p) * b[i - j], p);
        b[i] = norm(1LL * sum * scalar_power(static_cast<int>(i), p - 2, p), p);
    }
    return b;
}
Poly power(Poly a, std::uint64_t k, int p, std::size_t n) {
    Poly b(n); if (n) b[0] = 1;
    for (; k; k >>= 1, a = product(a, a, p, n)) if (k & 1) b = product(a, b, p, n);
    return b;
}
template <int P, int Root> void series_case(const Poly& input, std::size_t n, std::uint64_t k) {
    const auto saved = input;
    auto expected = input; expected.resize(n);
    for (auto& x : expected) x = norm(x, P);
    require((cp::fps_truncate<P>(input, n) == expected), "truncation and padding");
    if (!n || coefficient(input, 0, P)) require((cp::fps_inv<P, Root>(input, n) == inverse(input, P, n)), "inverse recurrence");
    Poly a = input; if (a.empty()) a.push_back(0); a[0] = 0;
    require((cp::fps_exp<P, Root>(a, n) == exponential(a, P, n)), "exp differential recurrence");
    a[0] = 1;
    const auto logarithm = cp::fps_log<P, Root>(a, n);
    require((exponential(logarithm, P, n) == cp::fps_truncate<P>(a, n)), "log via independent exp");
    require((cp::fps_pow<P, Root>(input, k, n) == power(input, k, P, n)), "power binary oracle");
    if constexpr (P != 2) {
        auto root = cp::fps_sqrt<P, Root>(input, n);
        std::size_t first = 0;
        while (first < n && !coefficient(input, first, P)) ++first;
        int lead = -1;
        if (first < n && P <= 97) for (int v = 0; v < P; ++v) if (v * v % P == coefficient(input, first, P)) { lead = v; break; }
        if (first == n) require(root.has_value(), "zero square root exists");
        else if (first & 1) require(!root, "odd valuation has no root");
        else if (P <= 97) require(root.has_value() == (lead >= 0), "square root existence by enumeration");
        if (root) {
            require(root->size() == n && product(*root, *root, P, n) == expected, "sqrt identity");
            if (first < n) require((*root)[first / 2] <= P / 2, "deterministic scalar root branch");
        }
    }
    require(input == saved, "input is immutable");
}
void exhaustive() {
    std::size_t cases = 0;
    for (int n = 0, count = 1; n <= 6; ++n, count *= 3) for (int code = 0; code < count; ++code) {
        Poly a(n); int v = code;
        for (auto& x : a) { x = v % 3 - 1; v /= 3; }
        for (std::size_t precision = 0; precision <= 8; ++precision) series_case<17, 3>(a, precision, code % 7);
        ++cases;
    }
    std::cout << cases << " ternary series at 9 precisions: coefficient recurrences, binary powers and enumerated roots passed\n";
}
void random_cases(std::mt19937_64& rng) {
    constexpr int p = 998244353;
    for (int trial = 0; trial < 240; ++trial) {
        const std::size_t n = rng() % 160, m = rng() % 120;
        Poly a(n), b(m);
        for (auto& x : a) x = static_cast<int>(static_cast<long long>(rng() % 4000000001ULL) - 2000000000);
        for (auto& x : b) x = static_cast<int>(static_cast<long long>(rng() % 4000000001ULL) - 2000000000);
        series_case<998244353, 3>(a, rng() % 161, trial % 3 ? rng() % 9 : rng());
        auto sum = cp::polynomial_add(a, b), difference = cp::polynomial_subtract(a, b);
        for (std::size_t i = 0; i < sum.size(); ++i) {
            require(sum[i] == norm(1LL * coefficient(a, i, p) + coefficient(b, i, p), p), "addition");
            require(difference[i] == norm(1LL * coefficient(a, i, p) - coefficient(b, i, p), p), "subtraction");
        }
        const auto size = n && m ? n + m - 1 : 0;
        require(cp::polynomial_multiply(a, b) == product(a, b, p, size), "NTT vs direct product");
        require(cp::polynomial_derivative(cp::polynomial_integral(a)) == cp::fps_truncate(a, a.size()), "calculus");
        if (b.empty()) b = {1};
        const auto [q, r] = cp::polynomial_divmod(a, b);
        auto reconstructed = cp::polynomial_add(cp::polynomial_multiply(q, b), r);
        require(cp::polynomial_trim(reconstructed) == cp::polynomial_trim(a), "division identity");
        require(r.size() < cp::polynomial_trim(b).size(), "remainder degree");
        const auto shift = static_cast<long long>(rng() % 20001) - 10000;
        auto shifted = cp::taylor_shift(a, shift);
        for (int x = -3; x <= 3; ++x) require(cp::polynomial_evaluate(shifted, x) == cp::polynomial_evaluate(a, x + shift), "Taylor shift evaluation");
        const std::size_t count = rng() % 97;
        Poly xs(count), ys(count);
        for (std::size_t i = 0; i < count; ++i) xs[i] = static_cast<int>(i) - 40;
        std::shuffle(xs.begin(), xs.end(), rng);
        ys = cp::multipoint_evaluation(a, xs);
        for (std::size_t i = 0; i < count; ++i) require(ys[i] == cp::polynomial_evaluate(a, xs[i]), "multipoint vs Horner");
        const auto interpolated = cp::polynomial_interpolation(xs, ys);
        require(interpolated.size() == count && cp::multipoint_evaluation(interpolated, xs) == ys, "interpolation and evaluation");
        if (n <= count) require(cp::polynomial_trim(interpolated) == cp::polynomial_trim(a), "interpolation recovers coefficients");
        if (count) { xs.back() = xs[0]; require(cp::multipoint_evaluation(a, xs).back() == ys[0], "duplicate evaluation points"); }
        Poly denominator = b; denominator[0] = 1;
        const std::size_t index = rng() % 180;
        const auto rational = product(a, inverse(denominator, p, index + 1), p, index + 1);
        require(cp::bostan_mori(a, denominator, index) == rational[index], "improper rational coefficient");
    }
    for (int trial = 0; trial < 300; ++trial) {
        const std::size_t d = 1 + rng() % 24, index = rng() % 150;
        Poly a(d), c(d);
        for (auto& x : a) x = static_cast<int>(rng() % p);
        for (auto& x : c) x = static_cast<int>(rng() % p);
        auto history = a; history.resize(std::max(d, index + 1));
        for (std::size_t i = d; i <= index; ++i) for (std::size_t j = 0; j < d; ++j) history[i] = norm(history[i] + 1LL * c[j] * history[i - 1 - j], p);
        require(cp::linear_recurrence_nth(a, c, index) == history[index], "direct recurrence");
    }
    std::cout << "240 random algebra/NTT/product-tree cases and 300 direct recurrences passed\n";
}
void boundaries() {
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    static_assert(cp::fps_max_size() == (1U << 22));
    static_assert(cp::fps_max_size<2, 1>() == 1);
    series_case<2, 1>({-1}, 1, maximum);
    series_case<3, 2>({2}, 1, maximum);
    series_case<97, 5>({0, 0, 4, 2, 9, 0}, 16, maximum);
    series_case<1224736769, 3>({std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 5}, 12, maximum);
    require(cp::fps_inv({}, 0).empty() && cp::fps_log({}, 0).empty() && cp::fps_exp({1}, 0).empty(), "zero precision skips algebraic preconditions");
    require(cp::fps_sqrt({1}, 0)->empty(), "empty optional root is success");
    require(cp::fps_exp({}, 5) == Poly({1, 0, 0, 0, 0}), "exp of empty series");
    require(cp::fps_pow({}, 0, 4) == Poly({1, 0, 0, 0}), "zero to zero");
    require(cp::fps_pow({0, 1}, maximum, 8) == Poly(8), "valuation shift overflow avoided");
    require(cp::fps_sqrt({0, 0, 0, 0, 1}, 4) == std::optional<Poly>(Poly(4)), "ignore high coefficients");
    require(!cp::fps_sqrt({0, 1}, 4), "odd valuation");
    require(!cp::fps_sqrt<17, 3>({3}, 8), "nonresidue");
    require(cp::polynomial_integral(Poly{}) == Poly({0}), "integral of empty");
    require(cp::polynomial_divmod({0, 0}, {1, 0}) == std::make_pair(Poly{}, Poly{}), "zero and trailing zeros");
    require(cp::polynomial_interpolation({4}, {0}) == Poly({0}), "interpolation retains length");
    require(cp::bostan_mori({1, 2, 3}, {2}, maximum) == 0, "polynomial coefficient beyond degree");
    require(cp::linear_recurrence_nth({7}, {3}, maximum) == norm(7LL * scalar_power(3, maximum, 998244353), 998244353), "uint64 recurrence index");
    require(cp::linear_recurrence_nth({4, 5}, {0, 0}, maximum) == 0, "vanishing recurrence");
    require(cp::multipoint_evaluation({1}, {}).empty(), "empty point set");
    rejects<std::domain_error>([] { cp::fps_inv({}, 1); });
    rejects<std::domain_error>([] { cp::fps_inv({0}, 1); });
    rejects<std::domain_error>([] { cp::fps_log({2}, 1); });
    rejects<std::domain_error>([] { cp::fps_exp({1}, 1); });
    rejects<std::domain_error>([] { cp::polynomial_integral<3>({1, 1, 1}); });
    rejects<std::domain_error>([] { cp::polynomial_divmod({1}, {0, 0}); });
    rejects<std::domain_error>([] { cp::polynomial_interpolation<17, 3>({1, 18}, {2, 2}); });
    rejects<std::invalid_argument>([] { cp::polynomial_interpolation({1}, {}); });
    rejects<std::domain_error>([] { cp::bostan_mori({1}, {0, 1}, 0); });
    rejects<std::domain_error>([] { cp::bostan_mori({}, {}, 1); });
    rejects<std::invalid_argument>([] { cp::linear_recurrence_nth({}, {}, 0); });
    rejects<std::invalid_argument>([] { cp::linear_recurrence_nth({1}, {1, 2}, 0); });
    rejects<std::length_error>([] { cp::fps_inv<17, 3>({1}, 9); });
    rejects<std::length_error>([] { cp::fps_log<17, 3>({1}, 9); });
    rejects<std::length_error>([] { cp::fps_exp<17, 3>({}, 9); });
    rejects<std::length_error>([] { cp::fps_pow<17, 3>({}, 0, 9); });
    rejects<std::length_error>([] { cp::fps_sqrt<17, 3>({}, 9); });
    rejects<std::length_error>([] { cp::taylor_shift<17, 3>(Poly(9), 0); });
    rejects<std::length_error>([] { cp::polynomial_divmod<17, 3>(Poly(9, 1), {1}); });
    rejects<std::length_error>([] { cp::multipoint_evaluation<17, 3>({1}, Poly(8)); });
    rejects<std::length_error>([] { cp::polynomial_interpolation<17, 3>(Poly(8), Poly(8)); });
    rejects<std::length_error>([] { cp::bostan_mori<17, 3>(Poly(9, 1), {1}, 0); });
    rejects<std::length_error>([] { cp::linear_recurrence_nth<17, 3>(Poly(8), Poly(8), 9); });
    rejects<std::length_error>([] { cp::polynomial_multiply<17, 3>(Poly(9), Poly(9)); });
    rejects<std::length_error>([] { cp::fps_truncate({}, std::numeric_limits<std::size_t>::max()); });
    std::cout << "empty/zero/trailing coefficients, small primes, integer extremes, uint64 indices and rejected conditions passed\n";
}
void large() {
    constexpr std::size_t n = 65536;
    constexpr int p = 998244353;
    const auto geometric = cp::fps_inv({1, -1}, n);
    require(geometric == Poly(n, 1), "large inverse");
    Poly linear(n); linear[1] = 1;
    const auto e = cp::fps_exp(linear, n);
    int factorial = 1;
    for (std::size_t i = 0; i < n; ++i) {
        if (i) factorial = norm(1LL * factorial * static_cast<int>(i), p);
        require(norm(1LL * factorial * e[i], p) == 1, "large exp inverse factorials");
    }
    require(cp::fps_log(e, n) == linear, "large log");
    const auto cube = cp::fps_pow({1, -1}, 3, n);
    require(cube[0] == 1 && cube[1] == p - 3 && cube[2] == 3 && cube[3] == p - 1 && std::count(cube.begin() + 4, cube.end(), 0) == static_cast<long>(n - 4), "large power");
    const auto root = cp::fps_sqrt({1, 2, 1}, n);
    require(root && (*root)[0] == 1 && (*root)[1] == 1 && std::count(root->begin() + 2, root->end(), 0) == static_cast<long>(n - 2), "large sqrt");
    Poly monomial(n); monomial.back() = 1;
    const auto shifted = cp::taylor_shift(monomial, 1);
    require(shifted.front() == 1 && shifted.back() == 1 && cp::polynomial_evaluate(shifted, 1) == scalar_power(2, n - 1, p), "large Taylor shift");
    const auto [q, r] = cp::polynomial_divmod(monomial, Poly(n / 2, 1));
    require(cp::polynomial_trim(cp::polynomial_add(cp::polynomial_multiply(q, Poly(n / 2, 1)), r)) == monomial, "large fast division");
    Poly xs(8192), a(8192);
    for (std::size_t i = 0; i < a.size(); ++i) { xs[i] = static_cast<int>(i); a[i] = static_cast<int>(i * i % p); }
    const auto ys = cp::multipoint_evaluation(a, xs);
    require(cp::polynomial_interpolation(xs, ys) == a, "large product tree roundtrip");
    for (std::size_t i : {0U, 1U, 100U, 8191U}) require(ys[i] == cp::polynomial_evaluate(a, xs[i]), "large Horner spot checks");
    Poly initial(4096), c(4096); initial[0] = 7; c.back() = 1;
    require(cp::linear_recurrence_nth(initial, c, std::numeric_limits<std::uint64_t>::max()) == 0, "large degree and huge index");
    require(cp::linear_recurrence_nth(initial, c, std::uint64_t(1) << 63) == 7, "large periodic recurrence");
    std::cout << "65536-term Newton/calculus/shift/division, 8192-point product tree, degree-4096 uint64 recurrence passed\n";
}
} // namespace
int main() {
    std::mt19937_64 rng(317017);
    boundaries(); exhaustive(); random_cases(rng); large();
}
