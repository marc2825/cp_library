#include "convolution/bluestein_fft.hpp"
#include "convolution/convolution_nimber.hpp"
#include "convolution/convolution_u64.hpp"
#include "convolution/fast_middle_product.hpp"
#include "convolution/multidimensional_cyclic_convolution.hpp"
#include "convolution/multiplicative_convolution.hpp"
#include "convolution/ntt64.hpp"
#include "convolution/ntt_avx2.hpp"
#include "convolution/ntt_doubling.hpp"
#include "convolution/rader_fft.hpp"
#include "convolution/transposed_convolution.hpp"
#include "test_common.hpp"
int main() {
    __extension__ using U = unsigned __int128;
    constexpr auto p = cp::ntt64_modulus;
    for (int t = 0; t < 100000; ++t) {
        auto a = random_engine() % p, b = random_engine() % p;
        CHECK(cp::ntt64_detail::mul(a, b) == std::uint64_t(U(a) * b % p));
        CHECK(cp::ntt64_detail::add(a, b) == std::uint64_t((U(a) + b) % p));
        CHECK(cp::ntt64_detail::sub(a, b) == std::uint64_t((U(a) + p - b) % p));
    }
    for (auto a : {UINT64_C(0), UINT64_C(1), p - 1, p - 2, UINT64_C(0xffffffff), UINT64_C(0x100000000)})
        for (auto b : {UINT64_C(0), UINT64_C(1), p - 1, p - 2})
            CHECK(cp::ntt64_detail::mul(a, b) == std::uint64_t(U(a) * b % p));
    for (int bits = 0; bits <= 15; ++bits) {
        std::size_t n = std::size_t(1) << bits;
        auto a = random_poly(n);
        auto ref = a;
        cp::ntt(ref);
        auto simd = a;
        cp::ntt_avx2(simd);
        CHECK(simd == ref);
        cp::ntt_avx2(simd, true);
        CHECK(simd == a);
        auto fallback = a;
        cp::ntt_avx2(fallback, false, true);
        CHECK(fallback == ref);
        auto doubled = ref;
        cp::ntt_doubling(doubled);
        a.resize(n * 2);
        cp::ntt(a);
        CHECK(a == doubled);
        std::vector<std::uint64_t> x(n);
        for (auto &v : x)
            v = random_engine();
        auto y = x;
        cp::ntt64(y);
        if (n <= 32) {
            auto root = cp::ntt64_detail::power(7, (p - 1) / n);
            for (std::size_t i = 0; i < n; ++i) {
                std::uint64_t v = 0, w = 1, step = cp::ntt64_detail::power(root, i);
                for (auto c : x) {
                    v = std::uint64_t((U(v) + U(c % p) * w) % p);
                    w = std::uint64_t(U(w) * step % p);
                }
                CHECK(y[i] == v);
            }
        }
        cp::ntt64(y, true);
        for (std::size_t i = 0; i < n; ++i)
            CHECK(y[i] == x[i] % p);
    }
    for (int t = 0; t < 100; ++t) {
        auto n = random_engine() % 130, m = random_engine() % 130;
        std::vector<std::uint64_t> a(n), b(m), c(n && m ? n + m - 1 : 0), gold = c;
        for (auto &x : a)
            x = random_engine();
        for (auto &x : b)
            x = random_engine();
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < m; ++j) {
                c[i + j] += a[i] * b[j];
                gold[i + j] = std::uint64_t((U(gold[i + j]) + U(a[i] % p) * (b[j] % p)) % p);
            }
        CHECK(cp::convolution_u64(a, b) == c);
        CHECK(cp::convolution_ntt64(a, b) == gold);
        if (t < 20) {
            std::fill(c.begin(), c.end(), 0);
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t j = 0; j < m; ++j)
                    c[i + j] ^= cp::nim_product(a[i], b[j]);
            CHECK(cp::convolution_nimber(a, b) == c);
        }
    }
    std::vector<std::uint64_t> all(10000, ~UINT64_C(0));
    auto wrap = cp::convolution_u64(all, all);
    for (std::size_t i = 0; i < wrap.size(); ++i)
        CHECK(wrap[i] == std::min(i + 1, wrap.size() - i));
    for (int n = 1; n <= 71; ++n) {
        std::vector<std::complex<double>> a(n), want(n);
        for (auto &x : a)
            x = {double(int(random_engine() % 21) - 10), double(int(random_engine() % 21) - 10)};
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                want[i] += a[j] * std::polar(1.0, 2 * std::acos(-1.0) * i * j / n);
        auto x = a;
        cp::bluestein_fft(x);
        for (int i = 0; i < n; ++i)
            CHECK(std::abs(x[i] - want[i]) < 1e-8);
        cp::bluestein_fft(x, true);
        for (int i = 0; i < n; ++i)
            CHECK(std::abs(x[i] - a[i]) < 1e-8);
        if (cp::convolution_detail::prime(n)) {
            x = a;
            cp::rader_fft(x);
            for (int i = 0; i < n; ++i)
                CHECK(std::abs(x[i] - want[i]) < 1e-8);
            cp::rader_fft(x, true);
            for (int i = 0; i < n; ++i)
                CHECK(std::abs(x[i] - a[i]) < 1e-8);
        }
    }
    for (int t = 0; t < 250; ++t) {
        auto a = random_poly(1 + random_engine() % 130), b = random_poly(1 + random_engine() % a.size());
        std::vector<int> want(a.size() - b.size() + 1);
        for (std::size_t i = 0; i < want.size(); ++i)
            for (std::size_t j = 0; j < b.size(); ++j)
                want[i] = norm(want[i] + static_cast<long long>(a[i + j]) * b[j]);
        CHECK(cp::fast_middle_product(a, b) == want);
        auto x = random_poly(1 + random_engine() % 60);
        auto output = random_engine() % (x.size() + b.size());
        cp::TransposedConvolutionPlan<> plan(b, x.size(), output);
        auto y = naive_product(b, x);
        y.resize(output);
        CHECK(plan.apply(x) == y);
        auto weights = random_poly(output);
        CHECK(dot(y, weights) == dot(x, plan.transpose(weights)));
    }
    for (int pindex : {2, 3, 5, 17, 67}) {
        auto a = random_poly(pindex), b = random_poly(pindex);
        std::vector<int> c(pindex);
        for (int i = 0; i < pindex; ++i)
            for (int j = 0; j < pindex; ++j)
                c[i * j % pindex] = norm(c[i * j % pindex] + static_cast<long long>(a[i]) * b[j]);
        CHECK(cp::multiplicative_convolution(a, b) == c);
    }
    for (auto shape : std::vector<std::vector<std::size_t>>{{}, {1}, {2, 3}, {3, 2, 2}}) {
        std::size_t size = 1;
        for (auto d : shape)
            size *= d;
        auto a = random_poly(size), b = random_poly(size);
        std::vector<int> c(size);
        for (std::size_t i = 0; i < size; ++i)
            for (std::size_t j = 0; j < size; ++j) {
                auto x = i, y = j;
                std::size_t k = 0, stride = 1;
                for (std::size_t d = shape.size(); d-- > 0;) {
                    k += ((x % shape[d] + y % shape[d]) % shape[d]) * stride;
                    stride *= shape[d];
                    x /= shape[d];
                    y /= shape[d];
                }
                c[k] = norm(c[k] + static_cast<long long>(a[i]) * b[j]);
            }
        CHECK(cp::multidimensional_cyclic_convolution(a, b, shape) == c);
    }
    rejects<std::invalid_argument>([] {
        std::vector<int> x(3);
        cp::ntt_avx2(x);
    });
    rejects<std::invalid_argument>([] {
        std::vector<std::uint64_t> x(3);
        cp::ntt64(x);
    });
    rejects<std::invalid_argument>([] {
        std::vector<std::complex<double>> x(4);
        cp::rader_fft(x);
    });
    rejects<std::invalid_argument>([] { cp::fast_middle_product({1}, {}); });
    std::cout << "transforms: direct DFTs, full-width CRT/Goldilocks, SIMD=" << cp::ntt_avx2_available()
              << ", adjoints and index-group products passed\n";
}
