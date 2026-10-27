#include "linear_algebra/characteristic_polynomial.hpp"
#include "linear_algebra/finite_extension_field.hpp"
#include "linear_algebra/frobenius_normal_form.hpp"
#include "linear_algebra/gf2_polynomial.hpp"
#include "linear_algebra/minimal_polynomial.hpp"
#include "test_common.hpp"
using cp::GF2Polynomial;
using cp::Matrix;
unsigned degree(unsigned a) {
    unsigned n = 0;
    while (a >>= 1)
        ++n;
    return n;
}
unsigned remainder(unsigned a, unsigned b) {
    while (a && degree(a) >= degree(b))
        a ^= b << (degree(a) - degree(b));
    return a;
}
unsigned product(unsigned a, unsigned b) {
    unsigned c = 0;
    for (unsigned i = 0; b; ++i, b >>= 1)
        if (b & 1)
            c ^= a << i;
    return c;
}
bool irreducible(unsigned a) {
    if (a < 2)
        return false;
    for (unsigned b = 2; degree(b) <= degree(a) / 2; ++b)
        if (!remainder(a, b))
            return false;
    return true;
}
template <int Mod> void field_check(std::vector<int> f) {
    cp::FiniteExtensionField<Mod> field(f);
    std::size_t q = 1;
    for (std::size_t i = 0; i < field.degree(); ++i)
        q *= Mod;
    auto decode = [&](std::size_t code) {
        std::vector<int> a(field.degree());
        for (auto &x : a) {
            x = code % Mod;
            code /= Mod;
        }
        return a;
    };
    for (std::size_t code = 0; code < q; ++code) {
        auto a = decode(code);
        CHECK(field.power(a, q) == a);
        CHECK(field.add(a, field.zero()) == a);
        if (code) {
            CHECK(field.multiply(a, field.inverse(a)) == field.one());
            CHECK(field.power(a, q - 1) == field.one());
        }
        for (std::size_t other = 0; other < q; ++other) {
            auto b = decode(other);
            std::vector<int> c(a.size() + b.size() - 1);
            for (std::size_t i = 0; i < a.size(); ++i)
                for (std::size_t j = 0; j < b.size(); ++j)
                    c[i + j] = (c[i + j] + a[i] * b[j]) % Mod;
            for (std::size_t i = c.size(); i-- > field.degree();) {
                int lead = c[i];
                for (std::size_t j = 0; j < f.size(); ++j)
                    c[i - field.degree() + j] = ((c[i - field.degree() + j] - lead * f[j]) % Mod + Mod) % Mod;
            }
            c.resize(field.degree());
            CHECK(field.multiply(a, b) == c);
        }
    }
    for (int t = 0; t < 150; ++t) {
        auto a = decode(l_random() % q), b = decode(l_random() % q), c = decode(l_random() % q);
        CHECK(field.multiply(a, field.add(b, c)) == field.add(field.multiply(a, b), field.multiply(a, c)));
    }
    throws<std::domain_error>([&] { field.inverse({}); });
}
template <int Mod> void frobenius_check(const Matrix<int> &a) {
    auto f = cp::frobenius_normal_form<Mod>(a);
    CHECK(modular_product<Mod>(a, f.basis) == modular_product<Mod>(f.basis, f.form));
    CHECK(modular_product<Mod>(f.basis, f.inverse_basis) == Matrix<int>::identity(a.rows()));
    std::vector<int> product{1};
    std::size_t offset = 0;
    for (std::size_t k = 0; k < f.invariant_factors.size(); ++k) {
        const auto &p = f.invariant_factors[k];
        CHECK(p.size() > 1 && p.back() == 1);
        if (k)
            CHECK(cp::finite_polynomial_detail::Algebra<Mod>::mod(p, f.invariant_factors[k - 1]).empty());
        std::vector<int> q(product.size() + p.size() - 1);
        for (std::size_t i = 0; i < product.size(); ++i)
            for (std::size_t j = 0; j < p.size(); ++j)
                q[i + j] = (q[i + j] + 1LL * product[i] * p[j]) % Mod;
        product = std::move(q);
        const auto d = p.size() - 1;
        for (std::size_t j = 0; j < d; ++j)
            for (std::size_t i = 0; i < a.rows(); ++i) {
                int want = 0;
                if (j + 1 < d && i == offset + j + 1)
                    want = 1;
                if (j + 1 == d && i >= offset && i < offset + d)
                    want = (Mod - p[i - offset]) % Mod;
                CHECK(f.form(i, offset + j) == want);
            }
        offset += d;
    }
    CHECK(offset == a.rows());
    CHECK(product == cp::characteristic_polynomial_mod<Mod>(a));
}
int main() {
    for (unsigned a = 0; a < 128; ++a)
        for (unsigned b = 0; b < 128; ++b) {
            GF2Polynomial x(a), y(b);
            CHECK((x + y).bits() == cp::BigInt(a ^ b));
            CHECK((x * y).bits() == cp::BigInt(product(a, b)));
            if (b) {
                auto qr = divmod(x, y);
                CHECK(qr.first * y + qr.second == x);
                CHECK(qr.second.bits() == cp::BigInt(remainder(a, b)));
            }
            auto bezout = cp::gf2_polynomial_extended_gcd(x, y);
            CHECK(bezout.first * x + bezout.second * y == bezout.gcd);
            unsigned u = a, v = b;
            while (v) {
                auto r = remainder(u, v);
                u = v;
                v = r;
            }
            CHECK(bezout.gcd.bits() == cp::BigInt(u));
        }
    for (unsigned f = 2; f < 512; ++f) {
        CHECK(cp::gf2_polynomial_is_irreducible(GF2Polynomial(f)) == irreducible(f));
        CHECK(cp::finite_field_polynomial_is_irreducible<2>(GF2Polynomial(f).coefficients()) ==
              irreducible(f));
        for (unsigned a = 0; a < 16; ++a) {
            auto inv = cp::gf2_polynomial_inverse(GF2Polynomial(a), GF2Polynomial(f));
            if (inv)
                CHECK((GF2Polynomial(a) * (*inv)) % GF2Polynomial(f) == GF2Polynomial(1));
            CHECK(bool(inv) ==
                  (cp::gf2_polynomial_gcd(GF2Polynomial(a), GF2Polynomial(f)) == GF2Polynomial(1)));
        }
    }
    for (int t = 0; t < 30; ++t) {
        std::vector<int> a(130 + l_random() % 160), b(70 + l_random() % 180), c(a.size() + b.size() - 1);
        for (auto &x : a)
            x = l_random() & 1;
        for (auto &x : b)
            x = l_random() & 1;
        b.back() = 1;
        for (std::size_t i = 0; i < a.size(); ++i)
            for (std::size_t j = 0; j < b.size(); ++j)
                c[i + j] ^= a[i] & b[j];
        auto x = GF2Polynomial::from_coefficients(a), y = GF2Polynomial::from_coefficients(b),
             z = GF2Polynomial::from_coefficients(c);
        CHECK(x * y == z);
        CHECK(z / y == x);
        CHECK(z % y == GF2Polynomial());
        CHECK(x.bits() == GF2Polynomial(x.bits()).bits());
    }
    field_check<2>({1, 1, 0, 1});
    field_check<3>({1, 0, 1});
    field_check<5>({2, 0, 1});
    field_check<3>({1, 1});
    cp::FiniteExtensionField<998244353> large({-3, 0, 1});
    CHECK(large.multiply({17, 91}, large.inverse({17, 91})) == large.one());
    CHECK(large.power({17, 91}, UINT64_C(998244353) * 998244353 - 1) == large.one());
    throws<std::invalid_argument>([] { cp::FiniteExtensionField<3> f({2, 0, 1}); });
    throws<std::invalid_argument>([] { cp::FiniteExtensionField<2> f({1}); });
    throws<std::invalid_argument>([] { GF2Polynomial p(cp::BigInt(-1)); });
    throws<std::domain_error>([] {
        auto q = GF2Polynomial(7) / GF2Polynomial();
        (void)q;
    });
    CHECK(GF2Polynomial().degree() == -1);
    CHECK(!GF2Polynomial(3).coefficient(100000));
    for (unsigned code = 0; code < 512; ++code) {
        Matrix<int> a(3, 3);
        auto x = code;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j) {
                a(i, j) = x % 2;
                x /= 2;
            }
        frobenius_check<2>(a);
    }
    for (unsigned code = 0; code < 81; ++code) {
        Matrix<int> a(2, 2);
        auto x = code;
        for (std::size_t i = 0; i < 2; ++i)
            for (std::size_t j = 0; j < 2; ++j) {
                a(i, j) = x % 3;
                x /= 3;
            }
        frobenius_check<3>(a);
    }
    for (int t = 0; t < 80; ++t) {
        auto n = l_random() % 7;
        frobenius_check<101>(random_matrix(n, n));
    }
    CHECK(cp::frobenius_normal_form<101>(Matrix<int>::identity(6)).invariant_factors.size() == 6);
    throws<std::invalid_argument>([] { cp::frobenius_normal_form(Matrix<int>(2, 3)); });
    std::cout << "finite algebra: 16,384 binary polynomial pairs, 510 irreducibility checks, >64-bit "
                 "products, finite field tables, 673 Frobenius similarities passed\n";
}
