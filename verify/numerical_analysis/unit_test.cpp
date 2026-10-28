#include "linear_algebra/gauss_jordan.hpp"
#include "numerical_analysis/all.hpp"
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <string>
using namespace cp;
using namespace cp::numerical;
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        if (!(x))                                                                                            \
            throw std::runtime_error(std::string("check line ") + std::to_string(__LINE__) + ": " #x);       \
    } while (false)
void near_impl(int line, long double a, long double b, long double e = 1e-10L) {
    if (!(std::abs(a - b) <= e * (1 + std::abs(b))))
        std::cerr << std::setprecision(25) << "near line " << line << ": " << a << " expected " << b
                  << " tolerance " << e << "\n";
    CHECK(std::abs(a - b) <= e * (1 + std::abs(b)));
}
#define near(...) near_impl(__LINE__, __VA_ARGS__)
template <class F> void rejects(F f) {
    bool thrown = false;
    try {
        f();
    } catch (const std::invalid_argument &) {
        thrown = true;
    }
    CHECK(thrown);
}
int main() {
    const auto inf = std::numeric_limits<long double>::infinity(),
               nan = std::numeric_limits<long double>::quiet_NaN(),
               huge = std::numeric_limits<long double>::max(),
               tiny = std::numeric_limits<long double>::denorm_min();
    CHECK(almost_equal(inf, inf) && !almost_equal(inf, -inf) && !almost_equal(nan, nan));
    CHECK(!almost_equal(huge, -huge, {0, 1.9L}) && almost_equal(huge, -huge, {0, 2}));
    CHECK(almost_equal(tiny, 0, {tiny, 0}) && !almost_equal(tiny, 0, {0, 0}));
    rejects([] { ErrorTolerance(-1, 0); });
    for (auto x : {0.L, tiny, -tiny, 1.L, -0.1L, std::ldexp(1.L, 10000), std::ldexp(1.L, -10000)})
        near(rational_to_long_double(rational_from_binary(x)), x, 1e-18L);
    CompensatedSum sum;
    sum.add(1e30L);
    sum.add(1);
    sum.add(-1e30L);
    CHECK(sum.value() == 1);
    sum.clear();
    CHECK(sum.value() == 0);
    KahanSum kahan;
    long double naive = 1;
    kahan.add(1);
    for (int i = 0; i < 10000; ++i) {
        naive += 1e-20L;
        kahan.add(1e-20L);
    }
    CHECK(kahan.value() > naive);
    near(kahan.value(), 1 + 1e-16L, 1e-18L);
    near(log_sum_exp({1000, 1000, -1000}), 1000 + std::log(2.L));
    CHECK(log_sum_exp({}) == -inf && log_sum_exp({-inf, -inf}) == -inf && log_sum_exp({0, inf}) == inf);
    near(log_sum_exp({0, -100}), std::exp(-100.L), 1e-45L);
    rejects([&] { log_sum_exp({inf, nan}); });
    auto q = quadratic_roots(1, -1e15L, 1);
    CHECK(q.status == Status::converged && q.roots.size() == 2);
    near(q.roots[0] / 1e-15L, 1, 1e-16L);
    near(q.roots[1] / 1e15L, 1, 1e-16L);
    CHECK(quadratic_roots(9, -6, 1).roots.size() == 1);
    CHECK(quadratic_roots(0, 0, 0).all_real);
    CHECK(quadratic_roots(0, 0, 1).roots.empty());
    near(quadratic_roots(0, 2, -6).roots[0], 3);
    CHECK(quadratic_roots(1, 0, 1).roots.empty());
    CHECK(quadratic_roots(1, tiny, 0).roots.size() == 2);
    CHECK(quadratic_roots(tiny, 1, 1).status == Status::non_finite);
    RootOptions o;
    o.x_tolerance = {1e-14L, 1e-14L};
    o.value_tolerance = 1e-16L;
    for (int n = 1; n <= 100; ++n) {
        auto f = [=](long double x) { return x * x - n; };
        auto d = [](long double x) { return 2 * x; };
        for (const auto &r : {bisection(f, 0, 11, o), brent(f, 0, 11, o), newton(f, d, 10, o)}) {
            CHECK(r.status == Status::converged || r.status == Status::stagnation);
            near(r.root, std::sqrt((long double)n), 1e-12L);
            if (r.bracketed) {
                CHECK(f(r.lower) * f(r.upper) <= 0);
            }
        }
    }
    CHECK(brent([](long double x) { return x * x + 1; }, -1, 1).status == Status::invalid_bracket);
    CHECK(newton([](long double) { return 1.L; }, [](long double) { return 0.L; }, 0).status ==
          Status::zero_derivative);
    CHECK(newton([](long double x) { return x * x * x - 2 * x + 2; },
                 [](long double x) { return 3 * x * x - 2; }, 0)
              .status == Status::max_iterations);
    CHECK(brent([&](long double) { return nan; }, 0, 1).status == Status::non_finite);
    RootOptions zero;
    zero.max_evaluations = 0;
    CHECK(bisection([](long double x) { return x; }, -1, 1, zero).status == Status::max_evaluations);
    zero = RootOptions{};
    zero.max_iterations = 0;
    CHECK(brent([](long double x) { return x * x - 2; }, 0, 2, zero).status == Status::max_iterations);
    CHECK(brent([](long double x) { return x; }, -huge, huge).root == 0);
    auto narrow = brent([](long double x) { return x < 1.000000000000000001L ? -1.L : 1.L; }, 1,
                        std::nextafter(1.L, 2.L), RootOptions{{0, 0}, 0, 100, 100});
    CHECK(narrow.status == Status::invalid_bracket || narrow.status == Status::stagnation);
    for (int n = 2; n <= 40; n += 2) {
        auto r = simpson([](long double x) { return x * x * x - 2 * x + 3; }, -2, 3, n);
        CHECK(r.status == Status::converged && !r.error_estimate);
        near(r.value, 26.25L);
    }
    for (int degree = 0; degree <= 8; ++degree) {
        auto r = adaptive_simpson([=](long double x) { return std::pow(x, degree); }, 0, 1);
        CHECK(r.status == Status::converged);
        near(r.value, 1.L / (degree + 1), 1e-10L);
        auto rev = adaptive_simpson([=](long double x) { return std::pow(x, degree); }, 1, 0);
        near(rev.value, -r.value);
    }
    near(adaptive_simpson([](long double x) { return std::sin(x); }, 0, std::acos(-1.L)).value, 2);
    near(adaptive_simpson([](long double x) { return std::exp(x); }, -1, 1).value,
         std::exp(1.L) - std::exp(-1.L));
    IntegrationOptions io;
    io.max_depth = 0;
    CHECK(adaptive_simpson([](long double x) { return std::exp(x); }, 0, 1, io).status == Status::max_depth);
    io = IntegrationOptions{};
    io.max_evaluations = 5;
    CHECK(adaptive_simpson([](long double x) { return std::exp(x); }, 0, 1, io).status ==
          Status::max_evaluations);
    io.max_evaluations = 0;
    CHECK(adaptive_simpson([](long double x) { return x; }, 1, 1, io).evaluations == 0);
    CHECK(adaptive_simpson([&](long double x) { return x == 0 ? inf : 1 / x; }, 0, 1).status ==
          Status::non_finite);
    CHECK(simpson([](long double) { return 1.L; }, 1, std::nextafter(1.L, 2.L), 2).status ==
          Status::stagnation);
    CHECK(adaptive_simpson([](long double) { return 1.L; }, 1, std::nextafter(1.L, 2.L)).status ==
          Status::stagnation);
    auto minwidth = simpson([](long double) { return 1e3000L; }, 0, 2 * tiny, 2);
    CHECK(minwidth.status == Status::converged);
    near(minwidth.value / (tiny * 1e3000L), 2, 1e-16L);
    auto adaptive_minwidth = adaptive_simpson([](long double) { return 1e3000L; }, 0, 4 * tiny);
    CHECK(adaptive_minwidth.status == Status::converged);
    near(adaptive_minwidth.value / (tiny * 1e3000L), 4, 1e-16L);
    // The estimator can miss cancellation even in a smooth degree-six polynomial.
    auto alias = [](long double x) { return 2 - 3 * x * x + 5 * std::pow(x, 4) - 4 * std::pow(x, 6); };
    auto missed = adaptive_simpson(alias, -1, 1);
    CHECK(missed.status == Status::converged && *missed.error_estimate == 0);
    near(missed.value, 8.L / 3);
    near(adaptive_simpson(alias, -1, 0).value + adaptive_simpson(alias, 0, 1).value, 20.L / 7);
    bool propagated = false;
    try {
        simpson([](long double) -> long double { throw std::overflow_error("user"); }, 0, 1, 2);
    } catch (const std::overflow_error &) {
        propagated = true;
    }
    CHECK(propagated);
    CHECK(real_polynomial_roots({0, 0}).all_real);
    CHECK(real_polynomial_roots({1}).intervals.empty());
    auto repeated = real_polynomial_roots({1, -4, 6, -4, 1});
    CHECK(repeated.status == Status::converged && repeated.distinct_roots == 1);
    near(repeated.intervals[0].value, 1);
    const BigRational delta(1, BigInt("1000000000000000000000000000000"));
    auto clustered = isolate_real_roots({BigRational(1) + delta, -BigRational(2) - delta, 1});
    CHECK(clustered.status == Status::converged && clustered.intervals.size() == 2);
    CHECK(clustered.intervals[0].upper <= clustered.intervals[1].lower);
    auto irr = real_polynomial_roots({-2, 0, 1});
    CHECK(irr.distinct_roots == 2);
    near(irr.intervals[0].value, -std::sqrt(2.L));
    near(irr.intervals[1].value, std::sqrt(2.L));
    PolynomialRootOptions po;
    po.max_subdivisions = 0;
    auto limited = real_polynomial_roots({-2, 0, 1}, po);
    CHECK(limited.status == Status::max_iterations && limited.intervals[0].count == 2);
    CHECK(real_polynomial_roots({1, 0, 1}).distinct_roots == 0);
    std::mt19937 rng(20261028);
    int compared = 0;
    for (int trial = 0; trial < 160; ++trial) {
        std::size_t n = 1 + rng() % 9;
        std::vector<long double> l(n - 1), d(n), u(n - 1), b(n);
        Matrix<BigRational> a(n, n);
        std::vector<BigRational> rhs(n);
        for (std::size_t i = 0; i < n; ++i) {
            d[i] = int(rng() % 9) - 4;
            b[i] = int(rng() % 9) - 4;
            a(i, i) = BigRational(int(d[i]));
            rhs[i] = BigRational(int(b[i]));
            if (i + 1 < n) {
                l[i] = int(rng() % 9) - 4;
                u[i] = int(rng() % 9) - 4;
                a(i + 1, i) = BigRational(int(l[i]));
                a(i, i + 1) = BigRational(int(u[i]));
            }
        }
        auto exact = gauss_jordan(a, rhs);
        TridiagonalFactorization factor(l, d, u, {0, 1e-14L});
        auto result = factor.solve(b);
        if (exact.status == LinearSystemStatus::unique) {
            CHECK(result.status == Status::converged);
            for (std::size_t i = 0; i < n; ++i)
                near(result.solution[i], rational_to_long_double(exact.particular[i]));
            ++compared;
        } else
            CHECK(result.status == Status::singular);
    }
    CHECK(compared > 100);
    TridiagonalFactorization pivot({1, 1}, {0, 2, 2}, {1, 1});
    CHECK(pivot.row_swaps() > 0);
    auto solution = pivot.solve({2, 8, 8});
    CHECK(solution.status == Status::converged);
    for (int i = 0; i < 3; ++i)
        near(solution.solution[i], i + 1);
    auto second = pivot.solve({4, 16, 16});
    for (int i = 0; i < 3; ++i)
        near(second.solution[i], 2 * (i + 1));
    CHECK(solve_tridiagonal({0}, {0, 0}, {0}, {1, 1}).status == Status::singular);
    CHECK(solve_tridiagonal({}, {}, {}, {}).solution.empty());
    rejects([] { TridiagonalFactorization({}, {1, 2}, {}); });
    const std::size_t n = 200000;
    std::vector<long double> l(n - 1, -1), d(n, 4), u(n - 1, -1), b(n, 2);
    b.front() = b.back() = 3;
    auto large = solve_tridiagonal(l, d, u, b);
    CHECK(large.status == Status::converged);
    for (auto x : large.solution)
        near(x, 1, 1e-15L);
    std::cout << "numerical: stability, 300 root comparisons, quadrature/limits, exact Sturm clusters, 160 "
                 "rational dense comparisons, 200000-row band solve passed\n";
}
