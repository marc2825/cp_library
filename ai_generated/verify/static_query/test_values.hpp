#pragma once
#include <algorithm>
#include <limits>
#include <string>

namespace static_test {
inline int minimum(int x, int y) { return std::min(x, y); }
inline int infinity() { return std::numeric_limits<int>::max(); }
inline std::string concatenate(std::string x, std::string y) { return x + y; }
inline std::string empty_string() { return {}; }
constexpr long long mod = 998244353;
struct Affine { long long a, b; };
// Left-to-right fold means apply the left function first, then the right.
inline Affine concatenate_affine(Affine left, Affine right) {
    return {right.a * left.a % mod, (right.a * left.b + right.b) % mod};
}
inline Affine identity() { return {1, 0}; }
inline long long evaluate(Affine f, long long x) { return (f.a * x + f.b) % mod; }
inline int operation_count = 0;
inline long long counted_sum(long long x, long long y) { ++operation_count; return x + y; }
inline long long zero() { return 0; }
struct First {
    int value;
    explicit First(int x) : value(x) {} // No default constructor.
};
inline First first(First x, First y) { return x.value == -1 ? y : x; }
inline First no_first() { return First(-1); }
struct Item { int key, payload; }; // No < or == operators.
struct ItemCompare {
    bool reverse;
    explicit ItemCompare(bool r) : reverse(r) {}
    bool operator()(const Item& x, const Item& y) const { return reverse ? x.key > y.key : x.key < y.key; }
};
struct CountingLess {
    int* calls;
    bool operator()(int x, int y) const { ++*calls; return x < y; }
};
}  // namespace static_test
