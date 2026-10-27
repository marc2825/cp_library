#include "fps/composition.hpp"
#include "fps/differential_equation.hpp"
#include "fps/fast_composition.hpp"
#include "fps/fractional_linear_transform.hpp"
#include "fps/multivariate_fps.hpp"
#include "fps/pow.hpp"
#include "fps/sharp_p_subset_sum.hpp"
#include "linear_recurrence/combinatorial_composition.hpp"
#include "test_common.hpp"
std::vector<int> tensor_product(const std::vector<int> &a, const std::vector<int> &b,
                                const std::vector<std::size_t> &shape) {
    std::vector<int> c(a.size());
    for (std::size_t i = 0; i < a.size(); ++i)
        for (std::size_t j = 0; j < b.size(); ++j) {
            auto x = i, y = j;
            bool ok = true;
            std::size_t target = 0, stride = 1;
            for (std::size_t d = shape.size(); d-- > 0;) {
                auto v = x % shape[d] + y % shape[d];
                if (v >= shape[d])
                    ok = false;
                target += v * stride;
                stride *= shape[d];
                x /= shape[d];
                y /= shape[d];
            }
            if (ok)
                c[target] = norm(c[target] + static_cast<long long>(a[i]) * b[j]);
        }
    return c;
}
int main() {
    for (int t = 0; t < 220; ++t) {
        std::size_t n = random_engine() % 100, count = random_engine() % 100;
        auto w = random_poly(n), g = random_poly(n);
        auto result = cp::power_projection(w, g, count);
        std::vector<int> power(n);
        if (n)
            power[0] = 1;
        for (std::size_t k = 0; k < count; ++k) {
            CHECK(result[k] == dot(w, power));
            power = naive_product(power, g);
            power.resize(n);
        }
        if (n)
            g[0] = 0;
        auto outer = random_poly(random_engine() % 120);
        CHECK(cp::fps_compose_fast(outer, g, n) == cp::fps_compose(outer, g, n));
        auto transform = cp::fps_fractional_linear_transform(outer, 3, n);
        std::vector<int> inner(n);
        int v = 1;
        for (std::size_t i = 1; i < n; ++i) {
            inner[i] = v;
            v = norm(v * 3LL);
        }
        CHECK(transform == cp::fps_compose(outer, inner, n));
        CHECK(cp::fps_fractional_linear_transform(transform, -3, n) == cp::fps_truncate(outer, n));
    }
    for (int t = 0; t < 100; ++t) {
        std::vector<std::size_t> weights(random_engine() % 15);
        for (auto &w : weights)
            w = random_engine() % 20;
        std::vector<int> count(51);
        for (std::size_t mask = 0; mask < (std::size_t(1) << weights.size()); ++mask) {
            std::size_t sum = 0;
            for (std::size_t j = 0; j < weights.size(); ++j)
                if (mask >> j & 1)
                    sum += weights[j];
            if (sum < count.size())
                ++count[sum];
        }
        CHECK(cp::sharp_p_subset_sum(weights, 50) == count);
    }
    for (auto shape : std::vector<std::vector<std::size_t>>{{}, {1}, {3, 4}, {2, 2, 3}}) {
        cp::MultivariateFPS<> ring(shape);
        for (int t = 0; t < 15; ++t) {
            auto a = random_poly(ring.size()), b = random_poly(ring.size());
            CHECK(ring.multiply(a, b) == tensor_product(a, b, shape));
            a[0] = 1;
            auto one = tensor_product(a, ring.inverse(a), shape);
            CHECK(one[0] == 1 && std::count(one.begin() + 1, one.end(), 0) == int(one.size() - 1));
            a[0] = 0;
            auto exp = ring.exponential(a);
            CHECK(ring.logarithm(exp) == a);
            std::vector<int> expected(ring.size()), power(ring.size());
            expected[0] = power[0] = 1;
            int invfact = 1;
            std::size_t degree = 0;
            for (auto s : shape)
                degree += s - 1;
            for (std::size_t k = 1; k <= degree; ++k) {
                power = tensor_product(power, a, shape);
                invfact = cp::fps_detail::Field<prime>::mul(invfact, cp::fps_detail::Field<prime>::inv(k));
                for (std::size_t i = 0; i < power.size(); ++i)
                    expected[i] = norm(expected[i] + static_cast<long long>(power[i]) * invfact);
            }
            CHECK(exp == expected);
        }
    }
    for (int t = 0; t < 80; ++t) {
        std::size_t n = 1 + random_engine() % 80;
        auto a = random_poly(n), b = random_poly(n);
        int initial = random_engine() % prime;
        auto y = cp::fps_solve_linear_ode(a, b, initial, n);
        auto rhs = naive_product(a, y);
        rhs.resize(n - 1);
        for (std::size_t i = 0; i < n - 1; ++i)
            rhs[i] = norm(rhs[i] + b[i]);
        CHECK(cp::polynomial_derivative(y) == rhs);
        CHECK(y[0] == initial);
        std::vector<std::vector<int>> coeff(1 + random_engine() % 5);
        for (auto &p : coeff)
            p = random_poly(6);
        auto z = cp::fps_solve_polynomial_ode(coeff, initial, n);
        std::vector<int> value(n - 1), power{1};
        for (const auto &p : coeff) {
            auto term = naive_product(p, power);
            term.resize(n - 1);
            for (std::size_t i = 0; i < n - 1; ++i)
                value[i] = norm(value[i] + term[i]);
            power = naive_product(power, z);
            power.resize(n - 1);
        }
        CHECK(cp::polynomial_derivative(z) == value);
        CHECK(z[0] == initial);
    }
    auto y = cp::fps_solve_linear_ode<17, 3>({0}, {1}, 3, 2);
    CHECK(y == std::vector<int>({3, 1}));
    for (auto op : {cp::LabelledConstruction::set, cp::LabelledConstruction::sequence,
                    cp::LabelledConstruction::cycle}) {
        auto result = cp::labelled_generating_function_compose({0, 1}, 20, op);
        int factorial = 1;
        for (int i = 0; i < 20; ++i) {
            if (i)
                factorial = norm(static_cast<long long>(factorial) * i);
            int want = op == cp::LabelledConstruction::set
                           ? cp::fps_detail::Field<prime>::inv(factorial)
                           : (op == cp::LabelledConstruction::sequence
                                  ? 1
                                  : (i ? cp::fps_detail::Field<prime>::inv(i) : 0));
            CHECK(result[i] == want);
        }
    }
    std::vector<int> g(8192), f(8192);
    g[1] = g[2] = 1;
    f[2000] = 1;
    auto fast = cp::fps_compose_fast(f, g, g.size());
    auto expected = cp::fps_pow(g, 2000, g.size());
    CHECK(fast == expected);
    rejects<std::domain_error>([] { cp::fps_compose_fast({1}, {1, 1}, 3); });
    rejects<std::length_error>([] { cp::fps_solve_linear_ode<17, 3>({1}, {}, 0, 18); });
    rejects<std::invalid_argument>([] { cp::MultivariateFPS<>({0}); });
    rejects<std::domain_error>([] { cp::MultivariateFPS<>({2, 2}).inverse({0, 0, 0, 0}); });
    std::cout << "FPS: naive powers/composition, subset enumeration, tensor ideals, ODE residuals and "
                 "8192-term composition passed\n";
}
