#include <algorithm>
#include <climits>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>
#include "number_theory_basic/all.hpp"
// 旧関数の正の小入力との互換比較。旧ヘッダには必要な標準ヘッダを先に渡す。
#include "number_theory/Extended_Euclidean.hpp"
#include "number_theory/Eratosthenes_Sieve.hpp"
#include "number_theory/factorization_basic.hpp"
#include "number_theory/enumerate_divisors.hpp"

#define CHECK(...) do { if (!(__VA_ARGS__)) { std::cerr << "check failed at " << __LINE__ << ": " << #__VA_ARGS__ << '\n'; std::abort(); } } while (false)
template <class Exception, class F> void throws(F f) { bool caught = false; try { f(); } catch (const Exception&) { caught = true; } CHECK(caught); }
bool naive_prime(int n) {
    if (n < 2) return false;
    for (int d = 2; d < n; ++d) if (n % d == 0) return false;
    return true;
}
void arithmetic() {
    for (int a = -100; a <= 100; ++a) for (int b = -100; b <= 100; ++b) {
        unsigned long long g = 0;
        for (int d = 1; d <= std::max(std::abs(a), std::abs(b)); ++d) if (a % d == 0 && b % d == 0) g = d;
        CHECK(cp::gcd(a, b) == g);
        const auto result = cp::extended_gcd(a, b);
        CHECK(result.gcd == g && a * result.x + b * result.y == static_cast<long long>(g));
        const auto l = cp::lcm(a, b);
        CHECK(l && *l == (g ? static_cast<unsigned long long>(std::abs(a * b)) / g : 0));
    }
    CHECK(cp::gcd(LLONG_MIN, 0) == (1ULL << 63));
    CHECK(cp::lcm(LLONG_MIN, LLONG_MIN) == (1ULL << 63));
    CHECK(cp::lcm(LLONG_MIN, 0) == 0);
    CHECK(!cp::lcm(LLONG_MIN, LLONG_MAX));
    CHECK(cp::lcm(LLONG_MAX, 2) == ULLONG_MAX - 1);
    CHECK(cp::lcm(3, static_cast<long long>(ULLONG_MAX / 3)) == ULLONG_MAX);
    for (int m = 1; m <= 60; ++m) for (int a = -80; a <= 80; ++a) {
        long long power = 1 % m, base = (a % m + m) % m;
        for (int e = 0; e <= 24; ++e) { CHECK(cp::mod_pow(a, e, m) == power); power = power * base % m; }
        std::optional<long long> inverse;
        for (int x = 0; x < m; ++x) if ((a * x % m + m) % m == 1 % m) { inverse = x; break; }
        CHECK(cp::mod_inverse(a, m) == inverse);
        for (int b = -20; b <= 20; ++b) CHECK(cp::mod_multiply(a, b, m) == (a * b % m + m) % m);
    }
    CHECK(cp::mod_pow(LLONG_MIN, LLONG_MAX, 1) == 0);
    CHECK(cp::mod_multiply(LLONG_MIN, LLONG_MIN, LLONG_MAX) == 1);
    CHECK(cp::mod_pow(LLONG_MAX - 1, LLONG_MAX, LLONG_MAX) == LLONG_MAX - 1);
    CHECK(cp::mod_inverse(LLONG_MIN, LLONG_MAX) == LLONG_MAX - 1);
    throws<std::invalid_argument>([]{cp::mod_pow(2, -1, 3);});
    for (long long bad : {0LL, -1LL, LLONG_MIN}) {
        throws<std::invalid_argument>([&]{cp::mod_pow(2, 0, bad);});
        throws<std::invalid_argument>([&]{cp::mod_multiply(0, 0, bad);});
        throws<std::invalid_argument>([&]{cp::mod_inverse(1, bad);});
    }
    std::cout << "40401 signed pairs; exhaustive small powers/products/inverses; 64-bit boundaries passed\n";
}
void sieves_and_factors() {
    cp::PrimeSieve sieve(10000); cp::SmallestPrimeFactor spf(10000);
    std::vector<int> primes;
    for (int n = 0; n <= 10000; ++n) {
        const bool prime = naive_prime(n);
        CHECK(sieve.is_prime(n) == prime && spf.is_prime(n) == prime);
        if (prime) primes.push_back(n);
        if (n <= 1) { CHECK(spf[n] == n); continue; }
        int smallest = 2; while (n % smallest) ++smallest;
        CHECK(spf[n] == smallest);
        std::vector<long long> ds; for (int d = 1; d <= n; ++d) if (n % d == 0) ds.push_back(d);
        CHECK(cp::divisors(n) == ds && enum_divisors(n) == ds);
        const auto factors = cp::prime_factorization(n);
        CHECK(factors == spf.factorize(n) && factors == factorization(n));
        long long product = 1, previous = 0, count = 1;
        for (auto [p, exponent] : factors) {
            CHECK(p > previous && naive_prime(static_cast<int>(p)) && exponent > 0);
            for (int i = 0; i < exponent; ++i) product *= p;
            previous = p; count *= exponent + 1;
        }
        CHECK(product == n && count == static_cast<long long>(ds.size()));
        long long x, y; CHECK(extGCD(n, n + 1, x, y) == 1 && n * x + (n + 1) * y == 1);
    }
    CHECK(sieve.primes() == primes && spf.primes() == primes && sieve.table() == Eratosthenes(10000));
    CHECK(cp::prime_factorization(1).empty() && spf.factorize(1).empty());
    CHECK(cp::divisors(1) == std::vector<long long>{1});
    for (int n : {0, 1, 2, 3, 4, 25, 49, 121}) {
        const cp::PrimeSieve s(n); const cp::SmallestPrimeFactor f(n);
        CHECK(s.limit() == n && f.limit() == n && s.table().size() == static_cast<std::size_t>(n + 1));
        CHECK(f.table().size() == s.table().size());
        for (int i = 0; i <= n; ++i) CHECK(s.is_prime(i) == naive_prime(i) && f.is_prime(i) == naive_prime(i));
        throws<std::out_of_range>([&]{s.is_prime(n + 1);});
        throws<std::out_of_range>([&]{f.factorize(n + 1);});
        throws<std::out_of_range>([&]{f.is_prime(n + 1);});
    }
    throws<std::invalid_argument>([]{cp::PrimeSieve s(-1);});
    throws<std::invalid_argument>([]{cp::SmallestPrimeFactor s(-1);});
    throws<std::out_of_range>([&]{sieve.is_prime(-1);});
    throws<std::out_of_range>([&]{spf[-1];});
    for (long long n : {0LL, -1LL, LLONG_MIN}) {
        throws<std::invalid_argument>([&]{cp::prime_factorization(n);});
        throws<std::invalid_argument>([&]{cp::divisors(n);});
        throws<std::invalid_argument>([&]{spf.factorize(static_cast<int>(n == LLONG_MIN ? -1 : n));});
    }
    CHECK(cp::prime_factorization(1LL << 62) == std::vector<std::pair<long long, int>>{{2, 62}});
    CHECK(cp::prime_factorization(LLONG_MAX) == std::vector<std::pair<long long, int>>{{7, 2}, {73, 1}, {127, 1}, {337, 1}, {92737, 1}, {649657, 1}});
    CHECK(cp::prime_factorization(1000000007LL * 1009) == std::vector<std::pair<long long, int>>{{1009, 1}, {1000000007, 1}});
    const auto ds = cp::divisors(1000000000000LL); CHECK(ds.size() == 169 && ds.front() == 1 && ds.back() == 1000000000000LL);
    cp::PrimeSieve large(2000000); cp::SmallestPrimeFactor fast(2000000);
    CHECK(large.primes().size() == 148933 && large.primes() == fast.primes());
    CHECK(fast.factorize(1999998) == cp::prime_factorization(1999998));
    std::cout << "10001 primality/SPF entries; 9999 divisor/factor lists; 2-million tables; legacy comparisons passed\n";
}
int main() { arithmetic(); sieves_and_factors(); }
