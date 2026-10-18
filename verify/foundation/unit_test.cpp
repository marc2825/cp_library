#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include "basic_utility/all.hpp"
#include "linear_algebra/matrix.hpp"
#include "linear_algebra/matrix_product.hpp"

void require(bool ok) { if (!ok) throw std::runtime_error("foundation check failed"); }
template <class E, class F> void throws(F f) {
    bool caught = false;
    try { f(); } catch (const E&) { caught = true; }
    require(caught);
}
template <class M> void modint_test() {
    const std::uint64_t m = M::mod();
    std::mt19937_64 rng(192026);
    for (int i = 0; i < 2000; ++i) {
        const auto a = rng(), b = rng();
        M x(a), y(b);
        const auto aa = a % m, bb = b % m;
        require(x.val() == aa && y.val() == bb);
        require((x + y).val() == (__uint128_t(aa) + bb) % m);
        require((x - y).val() == (__uint128_t(aa) + m - bb) % m);
        require((x * y).val() == __uint128_t(aa) * bb % m);
        require((-x).val() == (m - aa) % m);
        require((+x) == x && x.pow(0) == M(1));
        require(x.pow(5) == x*x*x*x*x);
        require((2 + x) == (x + 2) && (2 * x) == (x * 2));
        M z = x;
        require(z++ == x && z == x + 1);
        require(z-- == x + 1 && z == x);
        require(--(++z) == x);
        const auto inverse = y.try_inv();
        require(bool(inverse) == (std::gcd(bb, m) == 1));
        if (inverse) {
            require((y * *inverse) == M(1));
            require((x / y) * y == x);
            z /= y; require(z * y == x);
        } else throws<std::domain_error>([&] { (void)(x / y); });
    }
    const auto low = std::numeric_limits<__int128_t>::lowest();
    const auto high = std::numeric_limits<__uint128_t>::max();
    const auto residue = low % static_cast<__int128_t>(m);
    require(M(low).val() == static_cast<std::uint64_t>(residue < 0 ? residue + m : residue));
    require(M(high).val() == high % m);
    require(M(-1).val() == m - 1);
    require(M(1).pow(std::numeric_limits<std::uint64_t>::max()) == M(1));
    throws<std::invalid_argument>([] { (void)M(2).pow(-1); });
    throws<std::out_of_range>([] { (void)M(2).pow(__uint128_t(1) << 64); });
    std::istringstream in("-12345678901234567890123456789012345678901234567890 +7 12x");
    M value;
    in >> value;
    M expected = 0;
    for (char c : std::string("12345678901234567890123456789012345678901234567890")) expected = expected * 10 + c - '0';
    require(value == -expected);
    in >> value; require(value == M(7));
    in >> value; require(in.fail() && value == M(7));
    std::ostringstream out; out << value;
    require(out.str() == std::to_string(value.val()));
}

void reducers() {
    std::mt19937_64 rng(512);
    for (int i = 0; i < 100000; ++i) {
        const auto m = static_cast<std::uint32_t>(rng()) | 1U;
        cp::BarrettReduction barrett(m);
        const auto x = rng();
        const auto a = static_cast<std::uint32_t>(rng()), b = static_cast<std::uint32_t>(rng());
        require(barrett.reduce(x) == x % m);
        require(barrett.multiply(a, b) == std::uint64_t(a) * b % m);
        const auto n = (rng() >> 1) | 1;
        cp::MontgomeryReduction64 mont(n);
        require(mont.decode(mont.encode(x)) == x % n);
        require(mont.decode(mont.multiply_encoded(mont.encode(x), mont.encode(a))) == __uint128_t(x) * a % n);
    }
    for (std::uint32_t m : {1U, 2U, 3U, 4U, 2147483648U, 4294967294U, 4294967295U}) {
        cp::BarrettReduction b(m);
        for (std::uint64_t x : {0ULL, 1ULL, 4294967295ULL, 18446744073709551615ULL}) require(b.reduce(x) == x % m);
    }
    throws<std::invalid_argument>([] { cp::BarrettReduction b(0); });
    throws<std::invalid_argument>([] { cp::MontgomeryReduction64 b(0); });
    throws<std::invalid_argument>([] { cp::MontgomeryReduction64 b(2); });
    throws<std::invalid_argument>([] { cp::MontgomeryReduction64 b(1ULL << 63); });
    cp::MontgomeryReduction64 b(7);
    throws<std::out_of_range>([&] { b.reduce(__uint128_t(7) << 64); });
    throws<std::out_of_range>([&] { b.decode(7); });
    throws<std::out_of_range>([&] { b.multiply_encoded(1, 7); });
}

void generations() {
    using M = cp::DynamicModInt<1>;
    using N = cp::DynamicModInt<2>;
    M::set_mod(12); N::set_mod(7);
    M old(5); N other(5);
    M::set_mod(12); // Even re-setting the same modulus starts a new generation.
    throws<std::logic_error>([&] { (void)old.val(); });
    throws<std::logic_error>([&] { (void)(old == M(5)); });
    throws<std::logic_error>([&] { (void)old.pow(0); });
    throws<std::logic_error>([&] { (void)old.try_inv(); });
    throws<std::logic_error>([&] { (void)+old; });
    throws<std::logic_error>([&] { (void)(M(1) + old); });
    throws<std::logic_error>([&] { old *= M(1); });
    throws<std::logic_error>([&] { old -= M(1); });
    require(other.val() == 5);
    old = M(2); require(old.val() == 2);
    throws<std::invalid_argument>([] { M::set_mod(0); });
    require(old.val() == 2 && M::mod() == 12);
    for (auto m : {1U, 2U, 12U, 998244353U, 2147483648U, 4294967295U}) {
        M::set_mod(m); modint_test<M>();
    }
}

template <class T> void small_integers() {
    const int low = std::numeric_limits<T>::lowest(), high = std::numeric_limits<T>::max();
    for (int a = low; a <= high; ++a) for (int b = low; b <= high; ++b) {
        const T x = static_cast<T>(a), y = static_cast<T>(b);
        auto check = [&](std::optional<T> result, T saturated, int exact) {
            const bool fits = low <= exact && exact <= high;
            require(bool(result) == fits);
            if (fits) require(*result == exact);
            require(saturated == std::clamp(exact, low, high));
        };
        check(cp::checked_add(x, y), cp::saturating_add(x, y), a + b);
        check(cp::checked_sub(x, y), cp::saturating_sub(x, y), a - b);
        check(cp::checked_mul(x, y), cp::saturating_mul(x, y), a * b);
        if (b && !(a == low && b == -1)) {
            require(cp::floor_div(x, y) == std::floor(double(a) / b));
            require(cp::ceil_div(x, y) == std::ceil(double(a) / b));
        }
        if (a >= 0 && b >= 0) {
            require(cp::add_leq(x, y, static_cast<T>(high)) == (a + b <= high));
            require(cp::multiply_leq(x, y, static_cast<T>(high)) == (a * b <= high));
            require(cp::infinity_add(x, y, static_cast<T>(high)) == std::min(a + b, high));
        }
    }
}
void safety() {
    small_integers<std::int8_t>(); small_integers<std::uint8_t>();
    throws<std::invalid_argument>([] { cp::floor_div(1, 0); });
    throws<std::invalid_argument>([] { cp::ceil_div(1, 0); });
    throws<std::overflow_error>([] { cp::floor_div(std::numeric_limits<long long>::min(), -1LL); });
    throws<std::overflow_error>([] { cp::ceil_div(std::numeric_limits<__int128_t>::min(), __int128_t(-1)); });
    throws<std::invalid_argument>([] { cp::add_leq(-1, 0, 4); });
    throws<std::invalid_argument>([] { cp::multiply_leq(1, -1, 4); });
    throws<std::invalid_argument>([] { cp::infinity_add(1, 5, 4); });
    require(cp::multiply_leq(std::numeric_limits<unsigned long long>::max(), 0ULL, 0ULL));
}

struct File {
    std::FILE* p = std::tmpfile();
    File() { if (!p) throw std::runtime_error("tmpfile failed"); }
    explicit File(const std::string& s) : File() {
        require(std::fwrite(s.data(), 1, s.size(), p) == s.size()); std::rewind(p);
    }
    ~File() { std::fclose(p); }
};
template <class T> void roundtrip() {
    File file;
    const T lo = std::numeric_limits<T>::lowest(), hi = std::numeric_limits<T>::max();
    {
        cp::BasicFastPrinter<1> out(file.p);
        out.writeln(lo, hi, T(0), T(1)); out.flush();
        require(!out.failed());
    }
    std::rewind(file.p);
    cp::BasicFastScanner<1> in(file.p);
    T a, b, c, d;
    require(in.read(a,b,c,d) && a == lo && b == hi && c == 0 && d == 1);
    require(!in.read(d) && d == 1);
}
void io() {
    roundtrip<std::int8_t>(); roundtrip<std::uint8_t>(); roundtrip<std::int64_t>();
    roundtrip<std::uint64_t>(); roundtrip<__int128_t>(); roundtrip<__uint128_t>();
    roundtrip<float>(); roundtrip<double>(); roundtrip<long double>();
    for (const auto& s : {std::string(""), std::string(" \r\n\t"), std::string(200000, ' ')}) {
        File f(s); cp::BasicFastScanner<3> in(f.p); int x = 9; std::string token = "stay";
        require(!in.read(x) && x == 9); require(!in.read(token) && token == "stay");
    }
    for (const auto& s : {"-", "+", "x", "1x", "--1", "1.2"}) {
        File f(std::string(s) + " 7"); cp::BasicFastScanner<2> in(f.p); int x = 9;
        throws<std::invalid_argument>([&] { in.read(x); });
        require(x == 9 && in.read(x) && x == 7);
    }
    for (const auto& s : {"128", "-129", "9999999999999999999999999999999999"}) {
        File f(std::string(s) + " 7"); cp::BasicFastScanner<1> in(f.p); std::int8_t x = 9;
        throws<std::out_of_range>([&] { in.read(x); }); require(x == 9 && in.read(x) && x == 7);
    }
    { File f("-0 256"); cp::BasicFastScanner<1> in(f.p); std::uint8_t x = 9;
      throws<std::invalid_argument>([&] { in.read(x); });
      throws<std::out_of_range>([&] { in.read(x); }); require(x == 9); }
    for (const auto& s : {"1e99999", "1e-99999", "inf", "nan"}) {
        File f(s); cp::BasicFastScanner<3> in(f.p); double x = 9;
        throws<std::out_of_range>([&] { in.read(x); }); require(x == 9);
    }
    { File f("1e+bad"); cp::FastScanner in(f.p); double x = 1;
      throws<std::invalid_argument>([&] { in.read(x); }); require(x == 1); }
    { File f("\tAB " + std::string(200000, 'q') + " +42 -0 1.25e2");
      cp::BasicFastScanner<7> in(f.p); char a = 0, b = 0; std::string s; int x = 0, y = 0; double d = 0;
      require(in.read(a,b,s,x,y,d)); require(a == 'A' && b == 'B' && s == std::string(200000,'q') && x == 42 && y == 0 && d == 125);
      require(!in.read(x)); }
    { File f("1"); cp::FastScanner in(f.p); int a = 0, b = 2;
      require(!in.read(a,b) && a == 1 && b == 2); }
    { File f; { cp::BasicFastPrinter<3> out(f.p); out.write_fixed(1.25, 2); out.writeln("!", 5); out.writeln();
        throws<std::invalid_argument>([&] { out.write_fixed(0, -1); });
        throws<std::out_of_range>([&] { out.write(std::numeric_limits<double>::infinity()); }); }
      std::rewind(f.p); char text[64]{}; const auto n = std::fread(text,1,sizeof(text),f.p);
      require(std::string(text,n) == "1.25! 5\n\n"); }
    throws<std::invalid_argument>([] { cp::FastScanner in(nullptr); });
    throws<std::invalid_argument>([] { cp::FastPrinter out(nullptr); });
    // Linux error device: errors must be visible even when libc buffering delays them.
    if (auto* full = std::fopen("/dev/full", "w")) {
        { cp::FastPrinter out(full); out.write("test");
          throws<std::ios_base::failure>([&] { out.flush(); }); require(out.failed());
          throws<std::ios_base::failure>([&] { out.write('x'); }); }
        std::fclose(full);
    }
    if (auto* write_only = std::fopen("/dev/null", "w")) {
        cp::FastScanner in(write_only); int x = 9;
        throws<std::ios_base::failure>([&] { in.read(x); }); require(x == 9);
        std::fclose(write_only);
    }
}

int main() {
    modint_test<cp::StaticModInt<1>>(); modint_test<cp::StaticModInt<2>>();
    modint_test<cp::StaticModInt<12>>(); modint_test<cp::StaticModInt<4294967295U>>();
    modint_test<cp::ModInt998244353>(); modint_test<cp::ModInt61>();
    modint_test<cp::MontgomeryModInt64<1>>(); modint_test<cp::MontgomeryModInt64<9>>();
    modint_test<cp::MontgomeryModInt64<9223372036854775807ULL>>();
    generations(); reducers(); safety(); io();
    using M = cp::StaticModInt<101>;
    cp::Matrix<M> a{{1,2},{3,4}};
    auto b = cp::matrix_multiply(a,a);
    require(b(0,0) == M(7) && b(1,1) == M(22));
    std::cout << "foundation: modints, 100000 reducers, exhaustive 8-bit arithmetic, FILE I/O passed\n";
}
