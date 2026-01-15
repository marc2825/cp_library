#pragma once
#include <iostream>
#include <cassert>
#include <numeric>
#include <type_traits>
#include <vector>


namespace marclib {

namespace internal {

// @param n `0 <= n`
// @param m `1 <= m`
// @return `(x ** n) % m`
constexpr long long pow_mod_constexpr(long long x, long long n, int m) {
    if (m == 1) return 0;
    unsigned int _m = (unsigned int)(m);
    unsigned long long r = 1;
    unsigned long long y = safe_mod(x, m);
    while (n) {
        if (n & 1) r = (r * y) % _m;
        y = (y * y) % _m;
        n >>= 1;
    }
    return r;
}

constexpr long long safe_mod(long long x, long long m) {
    x %= m;
    if (x < 0) x += m;
    return x;
}

// @param b `1 <= b`
// @return pair(g, x) s.t. g = gcd(a, b), xa = g (mod b), 0 <= x < b/g
// When gcd(a,b) = 1, x is the modular inverse of a
constexpr std::pair<long long, long long> inv_gcd(long long a, long long b) {
    a = safe_mod(a, b);
    if (a == 0) return {b, 0};

    /// Euclidean Algorithm
    // Contracts:
    // [1] s - m0 * a = 0 (mod b)
    // [2] t - m1 * a = 0 (mod b)
    // [3] s * |m1| + t * |m0| <= b
    long long s = b, t = a;
    long long m0 = 0, m1 = 1;
    while (t) {
        long long u = s / t;
        s -= t * u;
        m0 -= m1 * u;  // |m1 * u| <= |m1| * s <= b

        // [3]:
        // (s - t * u) * |m1| + t * |m0 - m1 * u|
        // <= s * |m1| - t * u * |m1| + t * (|m0| + |m1| * u)
        // = s * |m1| + t * |m0| <= b

        auto tmp = s;
        s = t;
        t = tmp;
        tmp = m0;
        m0 = m1;
        m1 = tmp;
    }
    // by [3]: |m0| <= b/g
    // by g != b: |m0| < b/g
    if (m0 < 0) m0 += b / s;
    return {s, m0};
}

// Fast Primality Testing for Integers (32bit) using Miller-Rabin primality test
// @param n `0 <= n`
constexpr bool is_prime_constexpr(int n) {
    if (n <= 1) return false;
    if (n == 2 || n == 7 || n == 61) return true;
    if (n % 2 == 0) return false;
    long long d = n - 1;
    while (d % 2 == 0) d /= 2;
    constexpr long long bases[3] = {2, 7, 61};
    for (long long a : bases) {
        long long t = d;
        long long y = pow_mod_constexpr(a, t, n);
        while (t != n - 1 && y != 1 && y != n - 1) {
            y = y * y % n;
            t <<= 1;
        }
        if (y != n - 1 && t % 2 == 0) {
            return false;
        }
    }
    return true;
}
template <int n> constexpr bool is_prime = is_prime_constexpr(n);

// Fast modular multiplication by Barrett Reduction
// Reference: https://en.wikipedia.org/wiki/Barrett_reduction
// Approximates division by multiplying a precomputed inverse scaled by 2^64 and bit shifting.
struct barrett {
    unsigned int _m;
    unsigned long long im;

    // @param m `1 <= m`
    explicit barrett(unsigned int m) : _m(m), im((unsigned long long)(-1) / m + 1) {}

    // @return m
    unsigned int umod() const { return _m; }

    // @param a `0 <= a < m`
    // @param b `0 <= b < m`
    // @return `a * b % m`
    unsigned int mul(unsigned int a, unsigned int b) const {
        // [1] m = 1
        // a = b = im = 0, so okay

        // [2] m >= 2
        // im = ceil(2^64 / m)
        // -> im * m = 2^64 + r (0 <= r < m)
        // let z = a*b = c*m + d (0 <= c, d < m)
        // a*b * im = (c*m + d) * im = c*(im*m) + d*im = c*2^64 + c*r + d*im
        // c*r + d*im < m * m + m * im < m * m + 2^64 + m <= 2^64 + m * (m + 1) < 2^64 * 2
        // ((ab * im) >> 64) == c or c + 1
        unsigned long long z = a;
        z *= b;
#ifdef _MSC_VER
        unsigned long long x;
        _umul128(z, im, &x);
#else
        unsigned long long x =
            (unsigned long long)(((unsigned __int128)(z)*im) >> 64);
#endif
        unsigned long long y = x * _m;
        return (unsigned int)(z - y + (z < y ? _m : 0));
    }
};

} // namespace internal


/**
 * @brief コンパイル時にMODが決まっている高速なModInt (Static ModInt)
 */
template <int m, std::enable_if_t<(1 <= m)>* = nullptr>
struct static_modint {
    using mint = static_modint;

  public:
    static constexpr int mod() { return m; }
    
    // xに対してmodを取らずに、modint(x)を返す。（定数倍高速化用）
    static mint raw(int v) {
        mint x;
        x._v = v;
        return x;
    }

    constexpr static_modint() : _v(0) {}
    
    // 符号付き整数からのコンストラクタ
    template <class T, std::enable_if_t<std::is_signed<T>::value, int> = 0>
    constexpr static_modint(T v) {
        long long x = (long long)(v % (long long)(umod()));
        if (x < 0) x += umod();
        _v = (unsigned int)(x);
    }
    
    // 符号なし整数からのコンストラクタ
    template <class T, std::enable_if_t<std::is_unsigned<T>::value, int> = 0>
    constexpr static_modint(T v) {
        _v = (unsigned int)(v % umod());
    }

    constexpr unsigned int val() const { return _v; }

    constexpr mint& operator++() {
        _v++;
        if (_v == umod()) _v = 0;
        return *this;
    }
    constexpr mint& operator--() {
        if (_v == 0) _v = umod();
        _v--;
        return *this;
    }
    constexpr mint operator++(int) {
        mint result = *this;
        ++*this;
        return result;
    }
    constexpr mint operator--(int) {
        mint result = *this;
        --*this;
        return result;
    }

    constexpr mint& operator+=(const mint& rhs) {
        _v += rhs._v;
        if (_v >= umod()) _v -= umod();
        return *this;
    }
    constexpr mint& operator-=(const mint& rhs) {
        _v -= rhs._v;
        if (_v >= umod()) _v += umod();
        return *this;
    }
    constexpr mint& operator*=(const mint& rhs) {
        unsigned long long z = _v;
        z *= rhs._v;
        _v = (unsigned int)(z % umod());
        return *this;
    }
    constexpr mint& operator/=(const mint& rhs) { return *this = *this * rhs.inv(); }

    constexpr mint operator+() const { return *this; }
    constexpr mint operator-() const { return mint() - *this; }

    constexpr mint pow(long long n) const {
        assert(0 <= n);
        mint x = *this, r = 1;
        while (n) {
            if (n & 1) r *= x;
            x *= x;
            n >>= 1;
        }
        return r;
    }

    // 逆元計算
    constexpr mint inv() const {
        if (prime) {
            assert(_v);
            return pow(umod() - 2);
        } else {
            auto eg = internal::inv_gcd(_v, m);
            assert(eg.first == 1);
            return eg.second;
        }
    }

    friend constexpr mint operator+(const mint& lhs, const mint& rhs) { return mint(lhs) += rhs; }
    friend constexpr mint operator-(const mint& lhs, const mint& rhs) { return mint(lhs) -= rhs; }
    friend constexpr mint operator*(const mint& lhs, const mint& rhs) { return mint(lhs) *= rhs; }
    friend constexpr mint operator/(const mint& lhs, const mint& rhs) { return mint(lhs) /= rhs; }
    friend constexpr bool operator==(const mint& lhs, const mint& rhs) { return lhs._v == rhs._v; }
    friend constexpr bool operator!=(const mint& lhs, const mint& rhs) { return lhs._v != rhs._v; }

    // I/O ストリーム対応 (ACLにはない機能)
    friend std::ostream& operator<<(std::ostream& os, const mint& p) { return os << p._v; }
    friend std::istream& operator>>(std::istream& is, mint& p) {
        long long t;
        is >> t;
        p = mint(t);
        return is;
    }

  private:
    unsigned int _v;
    static constexpr unsigned int umod() { return m; }
    static constexpr bool prime = internal::is_prime_constexpr(m);
};

/**
 * @brief 実行時にMODが決まるModInt (Dynamic ModInt)
 * @tparam id 複数のMODを使い分けるための識別子
 */
template <int id>
struct dynamic_modint {
    using mint = dynamic_modint;

  public:
    static int mod() { return (int)(bt.umod()); }
    static void set_mod(int m) {
        assert(1 <= m);
        bt = internal::barrett(m);
    }
    static mint raw(int v) {
        mint x;
        x._v = v;
        return x;
    }

    dynamic_modint() : _v(0) {}
    
    template <class T, std::enable_if_t<std::is_signed<T>::value, int> = 0>
    dynamic_modint(T v) {
        long long x = (long long)(v % (long long)(mod()));
        if (x < 0) x += mod();
        _v = (unsigned int)(x);
    }
    template <class T, std::enable_if_t<std::is_unsigned<T>::value, int> = 0>
    dynamic_modint(T v) {
        _v = (unsigned int)(v % mod());
    }

    unsigned int val() const { return _v; }

    mint& operator++() {
        _v++;
        if (_v == umod()) _v = 0;
        return *this;
    }
    mint& operator--() {
        if (_v == 0) _v = umod();
        _v--;
        return *this;
    }
    mint operator++(int) {
        mint result = *this;
        ++*this;
        return result;
    }
    mint operator--(int) {
        mint result = *this;
        --*this;
        return result;
    }

    mint& operator+=(const mint& rhs) {
        _v += rhs._v;
        if (_v >= umod()) _v -= umod();
        return *this;
    }
    mint& operator-=(const mint& rhs) {
        _v += mod() - rhs._v;
        if (_v >= umod()) _v -= umod();
        return *this;
    }
    // Barrett Reduction による高速化
    mint& operator*=(const mint& rhs) {
        _v = bt.mul(_v, rhs._v);
        return *this;
    }
    mint& operator/=(const mint& rhs) { return *this = *this * rhs.inv(); }

    mint operator+() const { return *this; }
    mint operator-() const { return mint() - *this; }

    mint pow(long long n) const {
        assert(0 <= n);
        mint x = *this, r = 1;
        while (n) {
            if (n & 1) r *= x;
            x *= x;
            n >>= 1;
        }
        return r;
    }
    mint inv() const {
        auto eg = internal::inv_gcd(_v, mod());
        assert(eg.first == 1);
        return eg.second;
    }

    friend mint operator+(const mint& lhs, const mint& rhs) { return mint(lhs) += rhs; }
    friend mint operator-(const mint& lhs, const mint& rhs) { return mint(lhs) -= rhs; }
    friend mint operator*(const mint& lhs, const mint& rhs) { return mint(lhs) *= rhs; }
    friend mint operator/(const mint& lhs, const mint& rhs) { return mint(lhs) /= rhs; }
    friend bool operator==(const mint& lhs, const mint& rhs) { return lhs._v == rhs._v; }
    friend bool operator!=(const mint& lhs, const mint& rhs) { return lhs._v != rhs._v; }

    friend std::ostream& operator<<(std::ostream& os, const mint& p) { return os << p._v; }
    friend std::istream& operator>>(std::istream& is, mint& p) {
        long long t;
        is >> t;
        p = mint(t);
        return is;
    }

  private:
    unsigned int _v;
    static internal::barrett bt;
    static unsigned int umod() { return bt.umod(); }
};

template <int id> internal::barrett dynamic_modint<id>::bt(998244353);

using mint998 = static_modint;
using mint107 = static_modint;
using modint = dynamic_modint<-1>;

namespace literals {
    constexpr mint998 operator""_m998(unsigned long long n) { return mint998(n); }
    constexpr mint107 operator""_m107(unsigned long long n) { return mint107(n); }
}

} // namespace marclib