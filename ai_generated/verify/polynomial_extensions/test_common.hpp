#pragma once
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>
#define CHECK(...)                                                                                           \
    do {                                                                                                     \
        if (!(__VA_ARGS__)) {                                                                                \
            std::cerr << __FILE__ << ':' << __LINE__ << ": " << #__VA_ARGS__ << '\n';                        \
            std::abort();                                                                                    \
        }                                                                                                    \
    } while (false)
template <class E, class F> void rejects(F f) {
    bool ok = false;
    try {
        f();
    } catch (const E &) {
        ok = true;
    }
    CHECK(ok);
}
inline std::mt19937_64 random_engine(20261026);
inline constexpr int prime = 998244353;
inline int norm(long long x, int p = prime) {
    x %= p;
    return int(x < 0 ? x + p : x);
}
inline std::vector<int> naive_product(const std::vector<int> &a, const std::vector<int> &b, int p = prime) {
    if (a.empty() || b.empty())
        return {};
    std::vector<int> c(a.size() + b.size() - 1);
    for (std::size_t i = 0; i < a.size(); ++i)
        for (std::size_t j = 0; j < b.size(); ++j)
            c[i + j] = norm(c[i + j] + static_cast<long long>(a[i]) * b[j], p);
    return c;
}
inline std::vector<int> random_poly(std::size_t n, int p = prime) {
    std::vector<int> a(n);
    for (auto &x : a)
        x = random_engine() % p;
    return a;
}
inline int dot(const std::vector<int> &a, const std::vector<int> &b, int p = prime) {
    CHECK(a.size() == b.size());
    int r = 0;
    for (std::size_t i = 0; i < a.size(); ++i)
        r = norm(r + static_cast<long long>(a[i]) * b[i], p);
    return r;
}
