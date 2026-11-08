#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace cp {
struct GoldenSectionResult {
    long double position, value, left, right;
    std::size_t evaluations;
};
// Closed real interval. Strictly decreasing, minimum plateau, strictly increasing.
// Finite endpoints, width and function values required. At most iterations+4 calls.
template<class Function>
GoldenSectionResult golden_section_search(long double left, long double right, Function f, int iterations = 80) {
    if (!std::isfinite(left) || !std::isfinite(right) || left > right || !std::isfinite(right - left) || iterations < 0)
        throw std::invalid_argument("invalid golden-section domain or iteration count");
    std::size_t calls = 0;
    const auto evaluate = [&](long double x) {
        ++calls; const long double value = f(x);
        if (!std::isfinite(value)) throw std::domain_error("nonfinite objective value");
        return value;
    };
    long double best_x = left, best_y = evaluate(left);
    const auto observe = [&](long double x) {
        auto y = evaluate(x);
        if (y < best_y || (y == best_y && x < best_x)) { best_x = x; best_y = y; }
        return y;
    };
    if (left == right) return {best_x, best_y, left, right, calls};
    observe(right);
    constexpr long double ratio = 0.61803398874989484820458683436563811772L;
    long double x = right - ratio * (right - left), y = left + ratio * (right - left);
    auto fx = observe(x), fy = observe(y);
    for (int i = 0; i < iterations && left < x && x < y && y < right; ++i) {
        if (fx <= fy) {
            right = y; y = x; fy = fx;
            x = right - ratio * (right - left); fx = observe(x);
        } else {
            left = x; x = y; fx = fy;
            y = left + ratio * (right - left); fy = observe(y);
        }
    }
    return {best_x, best_y, left, right, calls};
}
template<class T> struct FibonacciSearchResult {
    std::int64_t position;
    T value;
    std::size_t evaluations;
};
// Integer [left,right), same shape assumption. Returns the leftmost minimum.
// Padding to Fibonacci length uses virtual +infinity; f is only called inside input.
template<class Function>
auto fibonacci_search(std::int64_t left, std::int64_t right, Function f)
    -> FibonacciSearchResult<std::decay_t<decltype(f(left))>> {
    using T = std::decay_t<decltype(f(left))>;
    if (left >= right) throw std::invalid_argument("empty Fibonacci-search domain");
    const __int128_t length = static_cast<__int128_t>(right) - left;
    std::vector<__int128_t> fib{0, 1, 1, 2};
    while (fib.back() < length + 1) fib.push_back(fib.back() + fib[fib.size() - 2]);
    std::size_t k = fib.size() - 1, calls = 0;
    std::optional<T> best;
    std::int64_t best_x = left;
    const auto evaluate = [&](const __int128_t x) -> std::optional<T> {
        if (x >= right) return std::nullopt;
        const auto pos = static_cast<std::int64_t>(x);
        T value = f(pos); ++calls;
        if constexpr (std::is_floating_point_v<T>)
            if (!std::isfinite(value)) throw std::domain_error("nonfinite objective value");
        if (!best || value < *best || (!( *best < value) && pos < best_x)) { best = value; best_x = pos; }
        return value;
    };
    if (length == 1) { evaluate(left); return {best_x, *best, calls}; }
    __int128_t a = static_cast<__int128_t>(left) - 1;
    __int128_t x = a + fib[k - 2], y = a + fib[k - 1];
    auto fx = evaluate(x), fy = evaluate(y);
    while (k > 3) {
        const bool take_left = !fy || (fx && !(*fy < *fx));
        --k;
        if (take_left) {
            y = x; fy = fx;
            if (k > 3) { x = a + fib[k - 2]; fx = evaluate(x); }
        } else {
            a = x; x = y; fx = fy;
            if (k > 3) { y = a + fib[k - 1]; fy = evaluate(y); }
        }
    }
    return {best_x, *best, calls};
}
} // namespace cp
