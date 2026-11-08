#pragma once
#include "ntt.hpp"
#if (defined(__x86_64__) || defined(__i386__)) && (defined(__GNUC__) || defined(__clang__))
#include <immintrin.h>
#define CP_NTT_HAS_AVX2_TARGET 1
#else
#define CP_NTT_HAS_AVX2_TARGET 0
#endif
namespace cp {
inline bool ntt_avx2_available() {
#if CP_NTT_HAS_AVX2_TARGET
    return __builtin_cpu_supports("avx2");
#else
    return false;
#endif
}
namespace ntt_avx2_detail {
#if CP_NTT_HAS_AVX2_TARGET
// Eight independent Montgomery products, canonical inputs/outputs. Mod < 2^30.
template <int Mod> __attribute__((target("avx2"))) __m256i multiply(__m256i a, __m256i b) {
    constexpr std::uint32_t neg = [] {
        std::uint32_t x = Mod;
        for (int i = 0; i < 5; ++i)
            x *= 2 - std::uint32_t(Mod) * x;
        return 0 - x;
    }();
    const auto p = _mm256_set1_epi32(Mod), ni = _mm256_set1_epi32(static_cast<int>(neg));
    auto e = _mm256_mul_epu32(a, b), o = _mm256_mul_epu32(_mm256_srli_epi64(a, 32), _mm256_srli_epi64(b, 32));
    auto em = _mm256_mul_epu32(e, ni), om = _mm256_mul_epu32(o, ni);
    e = _mm256_srli_epi64(_mm256_add_epi64(e, _mm256_mul_epu32(em, p)), 32);
    o = _mm256_srli_epi64(_mm256_add_epi64(o, _mm256_mul_epu32(om, p)), 32);
    auto z = _mm256_or_si256(e, _mm256_slli_epi64(o, 32));
    auto mask = _mm256_cmpgt_epi32(z, _mm256_set1_epi32(Mod - 1));
    return _mm256_sub_epi32(z, _mm256_and_si256(mask, p));
}
template <int Mod, int Root>
__attribute__((target("avx2"))) void transform(std::vector<int> &a, bool inverse) {
    constexpr std::uint64_t R = (UINT64_C(1) << 32) % Mod;
    const auto n = a.size();
    for (auto &x : a) {
        x %= Mod;
        if (x < 0)
            x += Mod;
        x = static_cast<int>(x * R % Mod);
    }
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        auto bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }
    const auto pm = _mm256_set1_epi32(Mod), top = _mm256_set1_epi32(Mod - 1), zero = _mm256_setzero_si256();
    for (std::size_t len = 2; len <= n; len *= 2) {
        auto root = convolution_detail::power(Root, (Mod - 1) / len, Mod);
        if (inverse)
            root = convolution_detail::power(root, Mod - 2, Mod);
        std::vector<int> w(len / 2), plain(len / 2);
        long long c = 1;
        for (std::size_t j = 0; j < len / 2; ++j) {
            plain[j] = static_cast<int>(c);
            w[j] = static_cast<int>(c * R % Mod);
            c = c * root % Mod;
        }
        for (std::size_t i = 0; i < n; i += len) {
            std::size_t j = 0;
            for (; j + 8 <= len / 2; j += 8) {
                auto u = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(a.data() + i + j));
                auto v = multiply<Mod>(
                    _mm256_loadu_si256(reinterpret_cast<const __m256i *>(a.data() + i + j + len / 2)),
                    _mm256_loadu_si256(reinterpret_cast<const __m256i *>(w.data() + j)));
                auto s = _mm256_add_epi32(u, v), d = _mm256_sub_epi32(u, v);
                s = _mm256_sub_epi32(s, _mm256_and_si256(_mm256_cmpgt_epi32(s, top), pm));
                d = _mm256_add_epi32(d, _mm256_and_si256(_mm256_cmpgt_epi32(zero, d), pm));
                _mm256_storeu_si256(reinterpret_cast<__m256i *>(a.data() + i + j), s);
                _mm256_storeu_si256(reinterpret_cast<__m256i *>(a.data() + i + j + len / 2), d);
            }
            for (; j < len / 2; ++j) {
                int u = a[i + j],
                    v = static_cast<int>(static_cast<long long>(a[i + j + len / 2]) * plain[j] % Mod);
                a[i + j] = u + v >= Mod ? u + v - Mod : u + v;
                a[i + j + len / 2] = u >= v ? u - v : u - v + Mod;
            }
        }
        if (len == n)
            break;
    }
    auto scale = convolution_detail::power(R, Mod - 2, Mod);
    if (inverse)
        scale = scale * convolution_detail::power(n, Mod - 2, Mod) % Mod;
    for (auto &x : a)
        x = static_cast<int>(x * scale % Mod);
}
#endif
} // namespace ntt_avx2_detail
// Runtime-dispatched AVX2; portable scalar fallback, no global -mavx2 requirement.
template <int Mod = 998244353, int Root = 3>
void ntt_avx2(std::vector<int> &a, bool inverse = false, bool force_scalar = false) {
    if (a.empty())
        return;
    if (!convolution_detail::power_of_two(a.size()))
        throw std::invalid_argument("NTT length must be a power of two");
    if (a.size() > ntt_max_size<Mod, Root>())
        throw std::length_error("NTT length exceeds modulus capacity");
#if CP_NTT_HAS_AVX2_TARGET
    if constexpr (Mod < (1 << 30)) {
        if (!force_scalar && ntt_avx2_available()) {
            ntt_avx2_detail::transform<Mod, Root>(a, inverse);
            return;
        }
    }
#else
    (void)force_scalar;
#endif
    ntt<Mod, Root>(a, inverse);
}
} // namespace cp
#undef CP_NTT_HAS_AVX2_TARGET
