#include "fps/composed_polynomial.hpp"
#include "fps/polynomial_extended_gcd.hpp"
#include "fps/polynomial_factorization.hpp"
#include "fps/polynomial_matrix.hpp"
#include "fps/polynomial_prefix_sum.hpp"
#include "fps/polynomial_resultant.hpp"
#include "fps/polynomial_roots.hpp"
#include "test_common.hpp"

template <int P> void factors(int max_degree) {
    using A = cp::finite_polynomial_detail::Algebra<P>;
    std::size_t cases = 0, range = 1;
    for (int degree = 0; degree <= max_degree; ++degree) {
        for (std::size_t mask = 0; mask < range; ++mask) {
            auto t = mask;
            std::vector<int> f(degree + 1, 1);
            for (int i = 0; i < degree; ++i) {
                f[i] = t % P;
                t /= P;
            }
            auto split = cp::polynomial_factorization<P>(f, mask);
            std::vector<int> reconstructed{split.leading_coefficient};
            for (const auto &factor : split.factors) {
                CHECK(factor.polynomial.back() == 1);
                CHECK(factor.multiplicity > 0);
                for (std::size_t k = 0; k < factor.multiplicity; ++k)
                    reconstructed = naive_product(reconstructed, factor.polynomial, P);
                std::size_t choices = 1;
                for (std::size_t d = 1; d <= (factor.polynomial.size() - 1) / 2; ++d) {
                    choices *= P;
                    for (std::size_t candidate = 0; candidate < choices; ++candidate) {
                        std::vector<int> divisor(d + 1, 1);
                        auto v = candidate;
                        for (std::size_t i = 0; i < d; ++i) {
                            divisor[i] = v % P;
                            v /= P;
                        }
                        CHECK(!A::mod(factor.polynomial, divisor).empty());
                    }
                }
            }
            CHECK(reconstructed == f);
            std::vector<int> roots;
            for (int x = 0; x < P; ++x)
                if (cp::polynomial_evaluate<P>(f, x) == 0)
                    roots.push_back(x);
            CHECK(cp::polynomial_roots<P>(f) == roots);
            ++cases;
        }
        range *= P;
    }
    std::cout << "factorization F_" << P << ": " << cases << " exhaustive polynomials\n";
}
int main() {
    factors<2>(8);
    factors<3>(5);
    factors<5>(4);
    using A = cp::finite_polynomial_detail::Algebra<prime>;
    using F = A::F;
    for (int test = 0; test < 250; ++test) {
        auto a = random_poly(random_engine() % 10), b = random_poly(random_engine() % 10);
        auto e = cp::polynomial_extended_gcd(a, b);
        CHECK(A::add(A::mul(a, e.first), A::mul(b, e.second)) == e.gcd);
        CHECK(e.gcd.empty() || e.gcd.back() == 1);
        if (!e.gcd.empty()) {
            CHECK(A::mod(a, e.gcd).empty());
            CHECK(A::mod(b, e.gcd).empty());
        }
        if (b.size() > 1 && b.back()) {
            auto inv = cp::polynomial_mod_inverse(a, b);
            CHECK(bool(inv) == (e.gcd == std::vector<int>{1}));
            if (inv)
                CHECK(A::mod(A::mul(a, *inv), b) == std::vector<int>{1});
        }
        auto s = cp::polynomial_prefix_sum(a);
        CHECK(cp::polynomial_evaluate(s, 0) == 0);
        for (int x = 0; x < 25; ++x)
            CHECK(norm(cp::polynomial_evaluate(s, x + 1) - cp::polynomial_evaluate(s, x)) ==
                  cp::polynomial_evaluate(a, x));
        // Independent Sylvester determinant, by modular Gaussian elimination.
        auto x = A::trim(a), y = A::trim(b);
        int expected = 0;
        if (!x.empty() && !y.empty()) {
            auto n = x.size() - 1, m = y.size() - 1;
            std::vector<std::vector<int>> matrix(n + m, std::vector<int>(n + m));
            for (std::size_t i = 0; i < m; ++i)
                for (std::size_t j = 0; j <= n; ++j)
                    matrix[i][i + j] = x[n - j];
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t j = 0; j <= m; ++j)
                    matrix[m + i][i + j] = y[m - j];
            expected = 1;
            for (std::size_t i = 0; i < n + m; ++i) {
                auto pivot = i;
                while (pivot < n + m && !matrix[pivot][i])
                    ++pivot;
                if (pivot == n + m) {
                    expected = 0;
                    break;
                }
                if (pivot != i) {
                    std::swap(matrix[pivot], matrix[i]);
                    expected = norm(-expected);
                }
                expected = F::mul(expected, matrix[i][i]);
                int inv = F::inv(matrix[i][i]);
                for (std::size_t row = i + 1; row < n + m; ++row) {
                    int scale = F::mul(matrix[row][i], inv);
                    for (std::size_t col = i; col < n + m; ++col)
                        matrix[row][col] = F::sub(matrix[row][col], F::mul(scale, matrix[i][col]));
                }
            }
        }
        CHECK(cp::polynomial_resultant(a, b) == expected);
    }
    for (int test = 0; test < 80; ++test) {
        auto roots1 = random_poly(1 + random_engine() % 5, 100),
             roots2 = random_poly(1 + random_engine() % 5, 100);
        std::vector<int> a{1}, b{1}, sum{1}, prod{1};
        for (int x : roots1)
            a = naive_product(a, {norm(-x), 1});
        for (int y : roots2)
            b = naive_product(b, {norm(-y), 1});
        for (int x : roots1)
            for (int y : roots2) {
                sum = naive_product(sum, {norm(-x - y), 1});
                prod = naive_product(prod, {norm(-x * y), 1});
            }
        CHECK(cp::polynomial_composed_sum(a, b) == sum);
        CHECK(cp::polynomial_composed_product(a, b) == prod);
        int disc = 1;
        for (std::size_t i = 0; i < roots1.size(); ++i)
            for (std::size_t j = 0; j < i; ++j)
                disc = F::mul(disc, F::mul(norm(roots1[i] - roots1[j]), norm(roots1[i] - roots1[j])));
        CHECK(cp::polynomial_discriminant(a) == disc);
        cp::PolynomialMatrix matrix(3, std::vector<std::vector<int>>(3));
        for (auto &row : matrix)
            for (auto &p : row)
                p = random_poly(3);
        std::vector<int> permutation{0, 1, 2}, det;
        do {
            int sign = 0;
            for (int i = 0; i < 3; ++i)
                for (int j = i + 1; j < 3; ++j)
                    sign ^= permutation[i] > permutation[j];
            std::vector<int> term{1};
            for (int i = 0; i < 3; ++i)
                term = naive_product(term, matrix[i][permutation[i]]);
            det = sign ? A::sub(det, term) : A::add(det, term);
        } while (std::next_permutation(permutation.begin(), permutation.end()));
        CHECK(cp::polynomial_matrix_determinant(matrix) == det);
        std::vector<std::vector<int>> expected_matrix(3, std::vector<int>(3));
        for (int i = 0; i < 3; ++i)
            expected_matrix[i][i] = 1;
        auto start = random_engine();
        int count = random_engine() % 40;
        for (int t = 0; t < count; ++t) {
            std::vector<std::vector<int>> next(3, std::vector<int>(3));
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    for (int k = 0; k < 3; ++k)
                        next[i][j] =
                            F::add(next[i][j],
                                   F::mul(cp::polynomial_evaluate(matrix[i][k], (start % prime + t) % prime),
                                          expected_matrix[k][j]));
            expected_matrix = next;
        }
        CHECK(cp::polynomial_matrix_prefix_product(matrix, start, count) == expected_matrix);
    }
    CHECK(cp::polynomial_factorization<2>({1, 0, 0, 0, 1}).factors[0].multiplicity == 4);
    CHECK(cp::polynomial_discriminant<2>({1, 0, 1}) == 0);
    CHECK(cp::polynomial_resultant({2}, {3}) == 1);
    CHECK(cp::polynomial_discriminant({}) == 0);
    CHECK(cp::polynomial_extended_gcd({}, {}).gcd.empty());
    rejects<std::domain_error>([] { cp::polynomial_factorization({}); });
    rejects<std::domain_error>([] { cp::polynomial_roots({0}); });
    rejects<std::invalid_argument>([] { cp::polynomial_mod_inverse({1}, {2}); });
    rejects<std::domain_error>([] { cp::polynomial_prefix_sum<3>({1, 2, 1}); });
    rejects<std::invalid_argument>([] { cp::polynomial_distinct_degree_factorization<3>({1, 2, 1}); });
    std::cout << "algebra: Bezout, Sylvester determinant, prefix differences, root pairs and matrix products "
                 "passed\n";
}
