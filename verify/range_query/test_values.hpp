#pragma once
#include <string>

namespace range_test {
constexpr long long mod = 998244353;
struct Affine { long long a, b; };
inline Affine compose(Affine f, Affine g) {
    return {f.a * g.a % mod, (f.a * g.b + f.b) % mod};
}
inline Affine identity() { return {1, 0}; }
inline long long apply(Affine f, long long x) { return (f.a * x + f.b) % mod; }
struct Sum { long long value; int length; };
inline Sum combine(Sum x, Sum y) { return {(x.value + y.value) % mod, x.length + y.length}; }
inline Sum empty() { return {0, 0}; }
inline Sum map_sum(Affine f, Sum x) {
    return {(f.a * x.value + f.b * x.length) % mod, x.length};
}
inline long long plus(long long x, long long y) { return x + y; }
inline long long zero() { return 0; }
inline std::string concat(std::string x, std::string y) { return x + y; }
inline std::string empty_string() { return {}; }
inline std::string shift(int f, std::string s) {
    for (char& c : s) c = char('a' + (c - 'a' + f) % 26);
    return s;
}
inline int add_shift(int f, int g) { return (f + g) % 26; }
inline int no_shift() { return 0; }
inline Affine concat_affine(Affine left, Affine right) { return compose(right, left); }
struct ModValue {
    long long value;
    ModValue(long long x = 0) : value((x % mod + mod) % mod) {}
    ModValue& operator+=(ModValue x) { value = (value + x.value) % mod; return *this; }
    friend ModValue operator-(ModValue x, ModValue y) { return {x.value - y.value}; }
    friend ModValue operator*(ModValue x, int k) { return {x.value * k}; }
};
}  // namespace range_test
