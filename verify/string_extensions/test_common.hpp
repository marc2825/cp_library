#pragma once
#include <algorithm>
#include <iostream>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        if (!(x))                                                                                            \
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #x);      \
    } while (false)
inline std::mt19937_64 random_s(20261027);
template <class E, class F> void throws(F f) {
    bool ok = false;
    try {
        f();
    } catch (const E &) {
        ok = true;
    }
    CHECK(ok);
}
inline std::vector<int> occurrences(const std::string &s, const std::string &p) {
    std::vector<int> out;
    for (std::size_t i = 0; i <= s.size(); ++i)
        if (s.substr(i, p.size()) == p)
            out.push_back(static_cast<int>(i));
    return out;
}
inline std::vector<std::string> words(int length, int alphabet = 2) {
    std::vector<std::string> out{std::string()};
    std::size_t begin = 0, end = 1;
    for (int i = 0; i < length; ++i) {
        for (auto k = begin; k < end; ++k)
            for (int c = 0; c < alphabet; ++c)
                out.push_back(out[k] + char(c));
        begin = end;
        end = out.size();
    }
    return out;
}
inline int lcs(const std::string &a, const std::string &b) {
    std::vector<int> dp(b.size() + 1);
    for (char c : a) {
        auto next = dp;
        for (std::size_t j = 0; j < b.size(); ++j)
            next[j + 1] = c == b[j] ? dp[j] + 1 : std::max(dp[j + 1], next[j]);
        dp = std::move(next);
    }
    return dp.back();
}
