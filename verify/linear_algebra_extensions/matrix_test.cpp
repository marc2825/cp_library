#include "basic_utility/static_modint.hpp"
#include "linear_algebra/characteristic_polynomial_fast.hpp"
#include "linear_algebra/hafnian.hpp"
#include "linear_algebra/permanent.hpp"
#include "linear_algebra/pfaffian.hpp"
#include "linear_algebra/semiring_matrix.hpp"
#include "test_common.hpp"
using cp::BigInt;
using cp::Matrix;
BigInt matching(const Matrix<int> &a, std::vector<std::size_t> vertices, bool signed_sum) {
    if (vertices.empty())
        return 1;
    auto first = vertices[0];
    BigInt sum = 0;
    for (std::size_t j = 1; j < vertices.size(); ++j) {
        auto rest = vertices;
        rest.erase(rest.begin() + j);
        rest.erase(rest.begin());
        BigInt term = BigInt(a(first, vertices[j])) * matching(a, rest, signed_sum);
        sum += (signed_sum && j % 2 == 0) ? -term : term;
    }
    return sum;
}
struct Words {
    using value_type = std::set<std::string>;
    value_type zero() const { return {}; }
    value_type one() const { return {""}; }
    value_type add(value_type a, const value_type &b) const {
        a.insert(b.begin(), b.end());
        return a;
    }
    value_type multiply(const value_type &a, const value_type &b) const {
        value_type c;
        for (const auto &x : a)
            for (const auto &y : b)
                c.insert(x + y);
        return c;
    }
};
template <int Mod> void characteristic_checks(int count) {
    for (int t = 0; t < count; ++t) {
        auto n = l_random() % 13;
        auto a = random_matrix(n, n, Mod == 998244353 ? 101 : Mod);
        auto result = cp::characteristic_polynomial_fast<Mod>(a, {3, l_random(), 1});
        CHECK(result.coefficients == cp::characteristic_polynomial_mod<Mod>(a));
        CHECK(result.attempts <= 3);
        if (result.used_krylov)
            CHECK(result.attempts > 0);
        auto repeat = cp::characteristic_polynomial_fast<Mod>(a, {0, 0, 1});
        CHECK(!repeat.used_krylov && repeat.coefficients == result.coefficients);
    }
}
int main() {
    for (unsigned code = 0; code < 512; ++code) {
        Matrix<BigInt> a(3, 3);
        auto x = code;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j) {
                a(i, j) = x & 1;
                x >>= 1;
            }
        CHECK(cp::permanent(a) == permutation_determinant(a, true));
    }
    for (int t = 0; t < 100; ++t) {
        auto n = 2 * (l_random() % 6);
        Matrix<int> skew(n, n), symmetric(n, n);
        Matrix<BigInt> exact(n, n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j) {
                int x = int(l_random() % 9) - 4;
                skew(i, j) = x;
                skew(j, i) = -x;
                symmetric(i, j) = symmetric(j, i) = x;
                exact(i, j) = exact(j, i) = x;
            }
        std::vector<std::size_t> vertices(n);
        std::iota(vertices.begin(), vertices.end(), 0);
        auto pf = matching(skew, vertices, true), hf = matching(symmetric, vertices, false);
        auto norm = [](BigInt x, int m) {
            x %= m;
            if (x < 0)
                x += m;
            return x.to_integral<int>();
        };
        CHECK(cp::pfaffian_mod<101>(skew) == norm(pf, 101));
        CHECK(cp::pfaffian_mod<2>(skew) == norm(pf, 2));
        CHECK(cp::hafnian(exact) == hf);
        if (n <= 6)
            CHECK(pf * pf == permutation_determinant(skew));
    }
    CHECK(cp::permanent(Matrix<BigInt>()) == 1);
    CHECK(cp::hafnian(Matrix<BigInt>()) == 1);
    CHECK(cp::hafnian(Matrix<int>(3, 3)) == 0);
    CHECK(cp::pfaffian_mod(Matrix<int>(3, 3)) == 0);
    Matrix<cp::StaticModInt<12>> composite{{1, 2}, {3, 4}};
    CHECK(cp::permanent(composite).val() == 10);
    CHECK(cp::hafnian(Matrix<cp::StaticModInt<12>>{{0, 7}, {7, 0}}).val() == 7);
    throws<std::invalid_argument>([] { cp::hafnian(Matrix<int>{{0, 1}, {2, 0}}); });
    throws<std::invalid_argument>([] { cp::pfaffian_mod(Matrix<int>{{1}}); });
    throws<std::invalid_argument>([] { cp::pfaffian_mod(Matrix<int>{{0, 1}, {1, 0}}); });
    throws<std::length_error>([] { cp::permanent(Matrix<int>(25, 25)); });
    throws<std::length_error>([] { cp::hafnian(Matrix<int>(23, 23)); });
    throws<std::overflow_error>([] { cp::permanent(Matrix<std::int8_t>{{100, 100}, {100, 100}}); });
    using O = std::optional<long long>;
    for (int t = 0; t < 180; ++t) {
        auto n = l_random() % 6;
        Matrix<O> a(n, n, std::nullopt);
        Matrix<std::uint8_t> b(n, n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                if (l_random() % 3) {
                    a(i, j) = int(l_random() % 13) - 6;
                    b(i, j) = 1;
                }
        auto exponent = l_random() % 9;
        for (bool maximum : {false, true}) {
            Matrix<O> want(n, n, std::nullopt);
            for (std::size_t i = 0; i < n; ++i)
                want(i, i) = 0;
            for (std::uint64_t k = 0; k < exponent; ++k) {
                Matrix<O> next(n, n, std::nullopt);
                for (std::size_t i = 0; i < n; ++i)
                    for (std::size_t j = 0; j < n; ++j)
                        for (std::size_t v = 0; v < n; ++v)
                            if (want(i, v) && a(v, j)) {
                                auto weight = *want(i, v) + *a(v, j);
                                if (!next(i, j) || (maximum ? weight > *next(i, j) : weight < *next(i, j)))
                                    next(i, j) = weight;
                            }
                want = std::move(next);
            }
            auto got = maximum ? cp::semiring_matrix_power(a, exponent, cp::MaxPlusSemiring<long long>{})
                               : cp::semiring_matrix_power(a, exponent, cp::MinPlusSemiring<long long>{});
            CHECK(got == want);
            auto boolean = cp::semiring_matrix_power(b, exponent, cp::BooleanSemiring{});
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t j = 0; j < n; ++j)
                    CHECK(bool(boolean(i, j)) == bool(want(i, j)));
        }
    }
    Words words;
    Matrix<Words::value_type> a(2, 2, words.zero());
    a(0, 1) = {"a"};
    a(1, 0) = {"b"};
    auto w = cp::semiring_matrix_power(a, 2, words);
    CHECK(w(0, 0) == Words::value_type{"ab"});
    CHECK(w(1, 1) == Words::value_type{"ba"});
    CHECK(cp::semiring_matrix_product(Matrix<O>(0, 3, std::nullopt), Matrix<O>(3, 4, std::nullopt),
                                      cp::MinPlusSemiring<long long>{})
              .cols() == 4);
    throws<std::overflow_error>([] {
        Matrix<std::optional<int>> a(1, 1, INT32_MAX);
        cp::semiring_matrix_power(a, 2, cp::MinPlusSemiring<int>{});
    });
    throws<std::invalid_argument>(
        [] { cp::semiring_matrix_power(Matrix<std::uint8_t>(2, 3), 2, cp::BooleanSemiring{}); });
    characteristic_checks<2>(100);
    characteristic_checks<3>(100);
    characteristic_checks<998244353>(100);
    for (std::size_t n : {33, 65, 100, 256}) {
        auto a = random_matrix(n, n, 1001);
        auto got = cp::characteristic_polynomial_fast(a, {2, 42, 16});
        CHECK(got.used_krylov);
        CHECK(got.coefficients == cp::characteristic_polynomial_mod(a));
    }
    for (auto n : {4, 33}) {
        auto a = Matrix<int>::identity(n);
        auto got = cp::characteristic_polynomial_fast(a, {2, 42, 2});
        CHECK(!got.used_krylov && got.attempts == 2);
        CHECK(got.coefficients == cp::characteristic_polynomial_mod(a));
    }
    throws<std::invalid_argument>([] { cp::characteristic_polynomial_fast(Matrix<int>(2, 3)); });
    throws<std::invalid_argument>([] { cp::characteristic_polynomial_fast(Matrix<int>(), {2, 0, 0}); });
    std::cout << "matrix algebra: 512 permanent enumerations, 100 signed/unsigned matching sums, 180 "
                 "semiring walks, 300 small-field and 256x256 fast characteristic comparisons passed\n";
}
