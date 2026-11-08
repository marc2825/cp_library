#include "linear_algebra/determinant_any_mod.hpp"
#include "linear_algebra/determinant_integer.hpp"
#include "linear_algebra/hermite_normal_form.hpp"
#include "linear_algebra/smith_normal_form.hpp"
#include "test_common.hpp"
using cp::BigInt;
using cp::Matrix;
BigInt minor_gcd(const Matrix<int> &a, std::size_t k) {
    BigInt g = 0;
    for (std::size_t x = 0; x < (std::size_t(1) << a.rows()); ++x) {
        auto rows = mask_indices(x);
        if (rows.size() != k)
            continue;
        for (std::size_t y = 0; y < (std::size_t(1) << a.cols()); ++y) {
            auto cols = mask_indices(y);
            if (cols.size() != k)
                continue;
            Matrix<int> b(k, k);
            for (std::size_t i = 0; i < k; ++i)
                for (std::size_t j = 0; j < k; ++j)
                    b(i, j) = a(rows[i], cols[j]);
            g = cp::gcd(g, permutation_determinant(b));
        }
    }
    return g;
}
int main() {
    for (int t = 0; t < 180; ++t) {
        auto a = random_matrix(l_random() % 5, l_random() % 5);
        auto big = cp::integer_matrix_detail::convert(a);
        auto s = cp::smith_normal_form(a);
        CHECK(cp::matrix_multiply(cp::matrix_multiply(s.left, big), s.right) == s.diagonal);
        CHECK(cp::abs(permutation_determinant(s.left)) == 1);
        CHECK(cp::abs(permutation_determinant(s.right)) == 1);
        BigInt product = 1;
        for (std::size_t i = 0; i < std::min(a.rows(), a.cols()); ++i) {
            if (i < s.rank) {
                CHECK(s.diagonal(i, i) > 0);
                if (i)
                    CHECK(s.diagonal(i, i) % s.diagonal(i - 1, i - 1) == 0);
            } else
                CHECK(s.diagonal(i, i) == 0);
            product *= s.diagonal(i, i);
            CHECK(product == minor_gcd(a, i + 1));
        }
        for (std::size_t i = 0; i < a.rows(); ++i)
            for (std::size_t j = 0; j < a.cols(); ++j)
                if (i != j)
                    CHECK(s.diagonal(i, j) == 0);
        auto h = cp::hermite_normal_form(a);
        CHECK(cp::matrix_multiply(h.left, big) == h.form);
        CHECK(cp::abs(permutation_determinant(h.left)) == 1);
        CHECK(h.pivot_columns.size() == s.rank);
        for (std::size_t i = 0; i < h.pivot_columns.size(); ++i) {
            auto c = h.pivot_columns[i];
            if (i)
                CHECK(c > h.pivot_columns[i - 1]);
            CHECK(h.form(i, c) > 0);
            for (std::size_t j = 0; j < c; ++j)
                CHECK(h.form(i, j) == 0);
            for (std::size_t r = 0; r < i; ++r)
                CHECK(h.form(r, c) >= 0 && h.form(r, c) < h.form(i, c));
            for (std::size_t r = i + 1; r < a.rows(); ++r)
                CHECK(h.form(r, c) == 0);
        }
        for (std::size_t i = s.rank; i < a.rows(); ++i)
            for (std::size_t j = 0; j < a.cols(); ++j)
                CHECK(h.form(i, j) == 0);
        if (a.rows() > 1) {
            auto changed = big;
            for (std::size_t j = 0; j < a.cols(); ++j)
                changed(0, j) += 7 * changed(1, j);
            CHECK(cp::hermite_normal_form(changed).form == h.form);
        }
        std::vector<BigInt> x(a.cols());
        for (auto &v : x)
            v = int(l_random() % 9) - 4;
        auto rhs = cp::matrix_vector_multiply(big, x);
        auto solution = cp::solve_integer_system(a, rhs);
        CHECK(solution.solvable);
        CHECK(cp::matrix_vector_multiply(big, solution.particular) == rhs);
        CHECK(solution.kernel_basis.size() == a.cols() - s.rank);
        for (const auto &v : solution.kernel_basis)
            CHECK(cp::matrix_vector_multiply(big, v) == std::vector<BigInt>(a.rows()));
    }
    std::size_t checked_solutions = 0;
    for (int t = 0; t < 360; ++t) {
        int mod = 1 + l_random() % 6;
        auto a = random_matrix(l_random() % 4, l_random() % 4);
        std::vector<int> b(a.rows());
        for (auto &x : b)
            x = int(l_random() % 13) - 6;
        auto s = cp::solve_congruence_system(a, b, BigInt(mod));
        auto normalize = [&](int x) { return (x % mod + mod) % mod; };
        std::set<std::vector<int>> expected, observed;
        std::size_t total = 1;
        for (std::size_t i = 0; i < a.cols(); ++i)
            total *= mod;
        for (std::size_t code = 0; code < total; ++code) {
            auto v = code;
            std::vector<int> x(a.cols());
            for (auto &y : x) {
                y = v % mod;
                v /= mod;
            }
            bool ok = true;
            for (std::size_t i = 0; i < a.rows(); ++i) {
                int dot = 0;
                for (std::size_t j = 0; j < a.cols(); ++j)
                    dot += a(i, j) * x[j];
                if (normalize(dot) != normalize(b[i]))
                    ok = false;
            }
            if (ok)
                expected.insert(x);
        }
        CHECK(s.solvable == !expected.empty());
        if (s.solvable) {
            std::size_t count = 1;
            for (const auto &order : s.orders)
                count *= order.to_integral<std::size_t>();
            for (std::size_t code = 0; code < count; ++code) {
                auto v = code;
                auto x = s.particular;
                for (std::size_t j = 0; j < s.orders.size(); ++j) {
                    auto order = s.orders[j].to_integral<std::size_t>();
                    for (std::size_t i = 0; i < x.size(); ++i)
                        x[i] += s.generators[j][i] * (v % order);
                    v /= order;
                }
                std::vector<int> y;
                for (const auto &z : x)
                    y.push_back(cp::integer_matrix_detail::residue(z, BigInt(mod)).to_integral<int>());
                CHECK(observed.insert(y).second);
            }
            CHECK(observed == expected);
            checked_solutions += count;
        }
    }
    const std::vector<std::uint64_t> moduli{
        1, 2, 4, 6, 12, 998244353, UINT64_C(1) << 63, UINT64_MAX, UINT64_MAX - 58};
    for (int t = 0; t < 160; ++t) {
        auto n = l_random() % 7;
        Matrix<BigInt> a(n, n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                a(i, j) = t % 3 ? BigInt(int(l_random() % 15) - 7) : BigInt(l_random()) - BigInt(l_random());
        auto d = permutation_determinant(a);
        CHECK(cp::determinant_integer(a) == d);
        for (auto m : moduli)
            CHECK(cp::determinant_any_mod(a, m) ==
                  cp::integer_matrix_detail::residue(d, BigInt(m)).to_integral<std::uint64_t>());
    }
    Matrix<std::uint64_t> wide{{UINT64_MAX, UINT64_MAX - 1}, {UINT64_MAX - 2, UINT64_MAX - 3}};
    CHECK(cp::determinant_any_mod(wide, UINT64_MAX) == UINT64_MAX - 2);
    BigInt huge("100000000000000000000000000000000000000000000000003");
    Matrix<BigInt> a{{huge, huge + 1}, {huge - 1, huge}};
    CHECK(cp::determinant_integer(a) == 1);
    auto s = cp::smith_normal_form(a);
    CHECK(s.diagonal == Matrix<BigInt>::identity(2));
    CHECK(!cp::solve_integer_system(Matrix<int>{{2}}, std::vector<int>{1}).solvable);
    CHECK(!cp::solve_integer_system(Matrix<int>(1, 0), std::vector<int>{1}).solvable);
    throws<std::invalid_argument>([] { cp::determinant_any_mod(Matrix<int>(2, 3), 7); });
    throws<std::invalid_argument>([] { cp::determinant_integer(Matrix<int>(2, 3)); });
    throws<std::invalid_argument>([] { cp::determinant_any_mod(Matrix<int>(), 0); });
    throws<std::invalid_argument>(
        [] { cp::solve_congruence_system(Matrix<int>(), std::vector<int>{}, BigInt(0)); });
    throws<std::invalid_argument>([] { cp::solve_integer_system(Matrix<int>(1, 2), std::vector<int>{}); });
    std::cout << "integer algebra: 180 determinantal-divisor/Hermite/lattice cases, 360 exhaustive modular "
                 "systems ("
              << checked_solutions << " solutions), 160 exact determinants and uint64 boundaries passed\n";
}
