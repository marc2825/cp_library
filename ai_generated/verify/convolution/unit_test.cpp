#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include "convolution/all.hpp"

namespace {
void require(bool good, const char* message) { if (!good) throw std::runtime_error(message); }
template <class E, class F> void rejects(F f) {
    bool caught = false;
    try { f(); } catch (const E&) { caught = true; }
    require(caught, "expected exception");
}
template <class T> std::vector<T> direct_product(const std::vector<T>& a, const std::vector<T>& b) {
    if (a.empty() || b.empty()) return {};
    std::vector<T> c(a.size() + b.size() - 1);
    for (std::size_t i = 0; i < a.size(); ++i) for (std::size_t j = 0; j < b.size(); ++j) c[i + j] += a[i] * b[j];
    return c;
}
template <class T> void close(T a, T b, long double tolerance = 1e-9L) {
    require(std::abs(a - b) <= tolerance * (1 + std::abs(b)), "floating point mismatch");
}
using Complex = std::complex<long double>;
std::vector<Complex> dft(const std::vector<Complex>& a, bool inverse) {
    std::vector<Complex> result(a.size());
    const auto pi = std::acos(-1.0L);
    for (std::size_t k = 0; k < a.size(); ++k) for (std::size_t j = 0; j < a.size(); ++j)
        result[k] += a[j] * std::polar(1.0L, (inverse ? -2 : 2) * pi * j * k / a.size());
    if (inverse) for (auto& x : result) x /= static_cast<long double>(a.size());
    return result;
}
long long pow_mod(long long a, long long b, int p) {
    long long r = 1;
    for (; b; b >>= 1, a = a * a % p) if (b & 1) r = r * a % p;
    return r;
}
template <int P, int Root> void check_ntt(std::mt19937_64& random) {
    require(cp::ntt_max_size<P, Root>() <= static_cast<std::size_t>(P - 1), "NTT capacity");
    for (std::size_t n = 1; n <= std::min<std::size_t>(64, cp::ntt_max_size<P, Root>()); n *= 2) {
        for (int trial = 0; trial < 10; ++trial) {
            std::vector<int> a(n), normalized(n), expected(n);
            for (std::size_t i = 0; i < n; ++i) {
                a[i] = static_cast<int>(static_cast<long long>(random() % 4000000001ULL) - 2000000000);
                normalized[i] = (static_cast<long long>(a[i]) % P + P) % P;
            }
            const auto w = pow_mod(Root, (P - 1) / static_cast<long long>(n), P);
            for (std::size_t k = 0; k < n; ++k) for (std::size_t j = 0; j < n; ++j)
                expected[k] = (expected[k] + normalized[j] * pow_mod(w, static_cast<long long>(j * k), P)) % P;
            auto forward = a;
            cp::ntt<P, Root>(forward); require(forward == expected, "NTT vs direct field DFT");
            cp::ntt<P, Root>(forward, true); require(forward == normalized, "NTT round trip");
            std::vector<int> inv(n);
            for (std::size_t k = 0; k < n; ++k) for (std::size_t j = 0; j < n; ++j) {
                auto term = normalized[j] * pow_mod(w, static_cast<long long>((n - j * k % n) % n), P) % P;
                inv[k] = (inv[k] + term * pow_mod(static_cast<long long>(n), P - 2, P)) % P;
            }
            cp::ntt<P, Root>(a, true); require(a == inv, "direct inverse NTT");
        }
    }
}
void exhaustive() {
    std::vector<std::vector<long long>> inputs{{}};
    for (int n = 1, size = 3; n <= 4; ++n, size *= 3) for (int code = 0; code < size; ++code) {
        std::vector<long long> a(n); auto x = code;
        for (auto& value : a) { value = x % 3 - 1; x /= 3; }
        inputs.push_back(a);
    }
    for (const auto& a : inputs) for (const auto& b : inputs) {
        const auto exact = direct_product(a, b);
        const auto mod = cp::convolution(a, b), small = cp::convolution<17, 3>(a, b);
        require(cp::convolution_ll(a, b) == exact, "exhaustive signed CRT");
        const auto approximate = cp::convolution_fft(std::vector<double>(a.begin(), a.end()), std::vector<double>(b.begin(), b.end()));
        require(mod.size() == exact.size() && small.size() == exact.size() && approximate.size() == exact.size(), "product sizes");
        for (std::size_t i = 0; i < exact.size(); ++i) {
            require(mod[i] == (exact[i] % 998244353 + 998244353) % 998244353, "exhaustive modular product");
            require(small[i] == (exact[i] % 17 + 17) % 17, "small prime product");
            close(approximate[i], static_cast<double>(exact[i]));
        }
    }
    std::cout << "14641 pairs of ternary arrays: modular / small-prime / signed CRT / real FFT vs direct product passed\n";
}
void floating(std::mt19937_64& random) {
    for (std::size_t n = 1; n <= 64; n *= 2) for (int trial = 0; trial < 12; ++trial) {
        std::vector<Complex> a(n);
        for (auto& x : a) x = Complex(static_cast<long double>(random() % 2001) / 8 - 125, static_cast<long double>(random() % 2001) / 8 - 125);
        for (bool inverse : {false, true}) {
            const auto expected = dft(a, inverse);
            auto actual = a; cp::fft(actual, inverse);
            for (std::size_t i = 0; i < n; ++i) close(actual[i], expected[i], 2e-14L);
            std::vector<std::complex<double>> ordinary(a.begin(), a.end()); cp::fft(ordinary, inverse);
            for (std::size_t i = 0; i < n; ++i) close(Complex(ordinary[i]), expected[i], 2e-11L);
        }
        auto recovered = a; cp::fft(recovered); cp::fft(recovered, true);
        for (std::size_t i = 0; i < n; ++i) close(recovered[i], a[i], 1e-14L);
        std::vector<double> real(n);
        for (auto& x : real) x = static_cast<double>(random() % 1001) / 16 - 30;
        const auto spectrum = cp::fft_real(real);
        std::vector<Complex> original(real.begin(), real.end()); const auto expected = dft(original, false);
        for (std::size_t i = 0; i < n; ++i) { close(Complex(spectrum[i]), expected[i], 1e-10L); close(spectrum[i], std::conj(spectrum[(n - i) % n])); }
    }
    for (int trial = 0; trial < 300; ++trial) {
        std::vector<Complex> a(random() % 65), b(random() % 65);
        for (auto* row : {&a, &b}) for (auto& x : *row) x = Complex(static_cast<long double>(random() % 2001) / 8 - 125, static_cast<long double>(random() % 2001) / 8 - 125);
        const auto exact = direct_product(a, b), actual = cp::convolution_fft(a, b);
        require(actual.size() == exact.size(), "complex product size");
        for (std::size_t i = 0; i < exact.size(); ++i) close(actual[i], exact[i], 2e-12L);
    }
    auto f = cp::fft_real(std::vector<float>{1, 2, 3, 4}); cp::fft(f, true);
    for (std::size_t i = 0; i < f.size(); ++i) close(f[i], std::complex<float>(static_cast<float>(i + 1)), 1e-5L);
    std::cout << "8 prime/root configurations, direct field DFT and inverse; complex DFT / inverse / conjugacy / float-double-long-double; 300 complex products passed\n";
}
void random_products(std::mt19937_64& random) {
    for (int trial = 0; trial < 400; ++trial) {
        std::vector<long long> a(1 + random() % 180), b(1 + random() % 180);
        for (auto* row : {&a, &b}) for (auto& x : *row) x = static_cast<long long>(random() % 2000001) - 1000000;
        const auto expected = direct_product(a, b), exact = cp::convolution_ll(a, b);
        const auto mod = cp::convolution(a, b);
        require(exact == expected, "random CRT");
        for (std::size_t i = 0; i < exact.size(); ++i) require(mod[i] == (expected[i] % 998244353 + 998244353) % 998244353, "random modular");
    }
    const auto hi = std::numeric_limits<long long>::max(), lo = std::numeric_limits<long long>::min();
    for (auto x : {lo, lo + 1, -4611686018427387904LL, -1224736769LL, -1LL, 0LL, 1LL, 167772161LL, 469762049LL, 1224736769LL, hi - 1, hi}) {
        require(cp::convolution_ll({x}, {1}) == std::vector<long long>{x}, "CRT signed boundary");
        if (x != lo) require(cp::convolution_ll({x}, {-1}) == std::vector<long long>{-x}, "CRT negative boundary");
    }
    require(cp::convolution_ll({lo, hi}, {0}) == std::vector<long long>({0, 0}), "zero skips absolute bound");
    require(cp::convolution_ll({hi}, {0, 1, 0}) == std::vector<long long>({0, hi, 0}), "smaller of two bounds");
    require(cp::convolution_ll({lo / 2, lo / 2}, {1}) == std::vector<long long>({lo / 2, lo / 2}), "absolute bound exactly 2^63");
    rejects<std::overflow_error>([&] { cp::convolution_ll({lo}, {-1}); });
    rejects<std::overflow_error>([&] { cp::convolution_ll({hi}, {2}); });
    rejects<std::overflow_error>([] { cp::convolution_ll({4294967296LL}, {4294967296LL}); });
    rejects<std::overflow_error>([&] { cp::convolution_ll({hi, hi, -hi}, {1, 1, 1}); }); // cancellation still requires conservative bound
    const std::vector<unsigned long long> unsigned_input{0, std::numeric_limits<unsigned long long>::max()};
    const auto mixed = cp::convolution(unsigned_input, std::vector<long long>{lo});
    require(mixed[0] == 0 && mixed[1] == static_cast<long long>(unsigned_input[1] % 998244353) * ((lo % 998244353 + 998244353) % 998244353) % 998244353, "mixed signed unsigned input");
    require(cp::convolution(std::vector<short>{-1}, std::vector<unsigned char>{2}) == std::vector<int>{998244351}, "small integral coefficient types");
    std::cout << "400 random products, mixed signed/unsigned types, all CRT signs and 64-bit limits, bound rejection passed\n";
}
void boundaries() {
    std::vector<int> empty; cp::ntt(empty); cp::ntt<2, 1>(empty, true);
    auto invalid = std::vector<int>{-1, 2, 3}, before = invalid;
    rejects<std::invalid_argument>([&] { cp::ntt(invalid); }); require(invalid == before, "invalid NTT unchanged");
    auto too_long = std::vector<int>(32, -1); before = too_long;
    rejects<std::length_error>([&] { cp::ntt<17, 3>(too_long); }); require(too_long == before, "overlong NTT unchanged");
    rejects<std::length_error>([] { cp::convolution<17, 3>(std::vector<int>(9), std::vector<int>(9)); });
    require(cp::convolution<17, 3>(std::vector<int>(), std::vector<int>(100)).empty(), "empty beats length");
    require(cp::convolution_ll({}, {1}).empty() && cp::convolution_ll({1}, {}).empty(), "empty integer products");
    const auto limit = std::numeric_limits<std::size_t>::max();
    rejects<std::length_error>([&] { cp::convolution_detail::shape(limit, 2, limit); });
    rejects<std::length_error>([&] { cp::convolution_detail::shape(limit, 1, limit); });
    require(cp::convolution_detail::shape(0, limit, 0) == std::make_pair(std::size_t(0), std::size_t(0)), "empty shape");
    require(cp::convolution_detail::shape(8, 9, 16) == std::make_pair(std::size_t(16), std::size_t(16)), "exact capacity");
    for (auto bad : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        for (bool imaginary : {false, true}) {
            std::vector<std::complex<double>> a{imaginary ? std::complex<double>(1, bad) : std::complex<double>(bad, 1)};
            rejects<std::invalid_argument>([&] { cp::fft(a); });
            rejects<std::invalid_argument>([&] { cp::convolution_fft(a, std::vector<std::complex<double>>{{1, 0}}); });
        }
        rejects<std::invalid_argument>([&] { cp::fft_real(std::vector<double>{bad}); });
    }
    std::vector<std::complex<double>> z; cp::fft(z, true);
    z.resize(3); rejects<std::invalid_argument>([&] { cp::fft(z); });
    const auto huge = std::numeric_limits<double>::max();
    z = {huge, huge}; rejects<std::overflow_error>([&] { cp::fft(z); });
    rejects<std::overflow_error>([&] { cp::convolution_fft(std::vector<double>{huge}, std::vector<double>{huge}); });
    require(cp::convolution_fft(std::vector<double>{}, std::vector<double>{huge}).empty(), "empty real convolution");
    std::cout << "empty/one/non-power-of-two/capacity/size_t bounds, NaN/Inf and floating overflow passed\n";
}
void large() {
    const std::size_t size = std::size_t(1) << 20;
    std::vector<int> a(size);
    for (std::size_t i = 0; i < size; ++i) a[i] = static_cast<int>((i * 3150167ULL + i * i) % 998244353);
    const auto before = a; cp::ntt(a); cp::ntt(a, true); require(a == before, "million point round trip");
    const int n = 100000;
    const auto product = cp::convolution(std::vector<int>(n, 1), std::vector<int>(n, 1));
    std::vector<long long> sign(n); for (int i = 0; i < n; ++i) sign[i] = i & 1 ? -1 : 1;
    const auto exact = cp::convolution_ll(sign, sign);
    for (int i = 0; i < 2 * n - 1; ++i) {
        const auto count = std::min(i + 1, 2 * n - 1 - i);
        require(product[i] == count && exact[i] == (i & 1 ? -count : count), "large triangle product");
    }
    const int m = 65537;
    const auto approximate = cp::convolution_fft(std::vector<double>(m, 1), std::vector<double>(m, 1));
    long double maximum_error = 0;
    for (int i = 0; i < 2 * m - 1; ++i) maximum_error = std::max(maximum_error, std::abs(static_cast<long double>(approximate[i]) - std::min(i + 1, 2 * m - 1 - i)));
    require(maximum_error < 1e-5L, "large FFT absolute error");
    std::cout << "2^20-point NTT; 100000x100000 modular/integer products; 65537x65537 real FFT max abs error " << maximum_error << " passed\n";
}
} // namespace
int main() {
    std::mt19937_64 random(316016);
    check_ntt<2, 1>(random); check_ntt<5, 2>(random); check_ntt<17, 3>(random); check_ntt<97, 5>(random);
    check_ntt<998244353, 3>(random); check_ntt<167772161, 3>(random); check_ntt<469762049, 3>(random); check_ntt<1224736769, 3>(random);
    exhaustive(); floating(random); random_products(random); boundaries(); large();
}
