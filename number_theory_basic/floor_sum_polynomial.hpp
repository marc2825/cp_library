#pragma once
#include <stdexcept>
#include <utility>
#include <vector>
#include "detail/advanced_arithmetic.hpp"

namespace cp {
namespace nt_detail {
template<class T, class Op> T monoid_power(T a, BigInt n, const T& identity, Op& op) {
    T result = identity;
    while (n != 0) {
        if (n % 2 != 0) result = op(result, a);
        n /= 2;
        if (n != 0) a = op(a, a);
    }
    return result;
}
// Product for i=1..n: U^(floor((a*i+b)/m)-floor((a*(i-1)+b)/m)) R.
template<class T, class Op> T euclid_word(BigInt n, BigInt m, BigInt a, BigInt b,
                                        T up, T right, const T& identity, Op& op) {
    if (n == 0) return identity;
    right = op(monoid_power(up, a / m, identity, op), right);
    a %= m;
    const BigInt height = (a * n + b) / m;
    if (height == 0) return monoid_power(right, n, identity, op);
    const BigInt first = (m - b - 1) / a, last = (m * height - b - 1) / a;
    const T prefix = op(monoid_power(right, first, identity, op), up);
    const T middle = euclid_word(height - 1, a, m, (m - b - 1) % a, right, up, identity, op);
    return op(op(prefix, middle), monoid_power(right, n - last, identity, op));
}
} // namespace nt_detail
// U^floor(b/m) R, then U^delta R for i=1..n-1. Noncommutative monoids allowed.
template<class T, class Op> T floor_word_product(const BigInt& n, const BigInt& m,
        const BigInt& a, const BigInt& b, T up, T right, T identity, Op op) {
    if (n < 0 || m <= 0 || a < 0 || b < 0) throw std::invalid_argument("floor word requires nonnegative parameters and m>0");
    if (n == 0) return identity;
    const T prefix = op(nt_detail::monoid_power(up, b / m, identity, op), right);
    return op(prefix, nt_detail::euclid_word(n - 1, m, a, b % m, up, right, identity, op));
}
// Exact sum i^p floor((a*i+b)/m)^q, 0<=i<n, p+q<=degree. Other entries are zero.
inline std::vector<std::vector<BigInt>> floor_sum_moments(const BigInt& n, const BigInt& m,
        const BigInt& a, const BigInt& b, unsigned degree, unsigned max_degree = 12) {
    if (n < 0 || m <= 0) throw std::invalid_argument("invalid floor sum parameters");
    if (degree > max_degree || degree > 64) throw std::length_error("floor moment degree budget");
    using Table = std::vector<std::vector<BigInt>>;
    const auto size = static_cast<std::size_t>(degree) + 1;
    Table choose(size, std::vector<BigInt>(size));
    for (unsigned i = 0; i <= degree; ++i) {
        choose[i][0] = choose[i][i] = 1;
        for (unsigned j = 1; j < i; ++j) choose[i][j] = choose[i - 1][j - 1] + choose[i - 1][j];
    }
    struct Segment { BigInt x, y; Table sum; };
    Segment identity{0, 0, Table(size, std::vector<BigInt>(size))};
    auto op = [&](const Segment& left, const Segment& right) {
        Segment result{left.x + right.x, left.y + right.y, left.sum};
        std::vector<BigInt> xp(size, 1), yp(size, 1);
        for (unsigned i = 1; i <= degree; ++i) { xp[i] = xp[i - 1] * left.x; yp[i] = yp[i - 1] * left.y; }
        for (unsigned p = 0; p <= degree; ++p) for (unsigned q = 0; p + q <= degree; ++q)
            for (unsigned i = 0; i <= p; ++i) for (unsigned j = 0; j <= q; ++j)
                result.sum[p][q] += choose[p][i] * choose[q][j] * xp[p - i] * yp[q - j] * right.sum[i][j];
        return result;
    };
    Segment up = identity, right = identity, initial = identity;
    up.y = 1; right.x = 1; right.y = nt_detail::floor_div(a, m); right.sum[0][0] = 1;
    initial.y = nt_detail::floor_div(b, m);
    const BigInt ar = a - right.y * m, br = b - initial.y * m;
    return op(initial, floor_word_product(n, m, ar, br, up, right, identity, op)).sum;
}
// Arbitrary precision ordinary floor sum, including negative a,b.
inline BigInt floor_sum_big(BigInt n, BigInt m, BigInt a, BigInt b) {
    if (n < 0 || m <= 0) throw std::invalid_argument("invalid floor sum parameters");
    BigInt result = 0;
    for (;;) {
        const BigInt qa = nt_detail::floor_div(a, m), qb = nt_detail::floor_div(b, m);
        result += qa * n * (n - 1) / 2 + qb * n;
        a -= qa * m; b -= qb * m;
        const BigInt top = a * n + b;
        if (top < m) break;
        n = top / m; b = top % m; std::swap(a, m);
    }
    return result;
}
} // namespace cp
