#pragma once
#include "../basic_utility/safe_integer.hpp"
#include "search_common.hpp"
#include <algorithm>
#include <vector>
namespace cp {
namespace tsp_detail {
template <class T> T add(T a, T b) {
    heuristic_detail::validate_score(a);
    heuristic_detail::validate_score(b);
    static_assert(std::is_signed_v<T>, "TSP delta needs a signed score");
    if constexpr (std::is_integral_v<T>) {
        auto x = checked_add(a, b);
        if (!x)
            throw std::overflow_error("TSP score overflow");
        return *x;
    } else {
        T x = a + b;
        heuristic_detail::validate_score(x);
        return x;
    }
}
template <class T> T sub(T a, T b) {
    heuristic_detail::validate_score(a);
    heuristic_detail::validate_score(b);
    if constexpr (std::is_integral_v<T>) {
        auto x = checked_sub(a, b);
        if (!x)
            throw std::overflow_error("TSP delta overflow");
        return *x;
    } else {
        T x = a - b;
        heuristic_detail::validate_score(x);
        return x;
    }
}
inline void interval(std::size_t n, std::size_t first, std::size_t count) {
    if (first > n || count > n - first)
        throw std::out_of_range("TSP interval");
}
} // namespace tsp_detail
template <class Distance>
auto tsp_tour_cost(const std::vector<int> &tour, Distance distance)
    -> std::decay_t<std::invoke_result_t<Distance &, int, int>> {
    using Cost = std::decay_t<std::invoke_result_t<Distance &, int, int>>;
    Cost total = 0;
    for (std::size_t i = 0; i < tour.size(); ++i)
        total = tsp_detail::add(total, distance(tour[i], tour[(i + 1) % tour.size()]));
    return total;
}
struct TspTwoOpt {
    std::size_t first, last;
}; // reverse [first,last)
inline void apply_two_opt(std::vector<int> &tour, TspTwoOpt move) {
    if (move.last < move.first)
        throw std::out_of_range("TSP reverse interval");
    tsp_detail::interval(tour.size(), move.first, move.last - move.first);
    std::reverse(tour.begin() + move.first, tour.begin() + move.last);
}
inline void undo_two_opt(std::vector<int> &tour, TspTwoOpt move) noexcept {
    std::reverse(tour.begin() + move.first, tour.begin() + move.last);
}
template <class Distance>
auto two_opt_delta(const std::vector<int> &tour, TspTwoOpt move, Distance distance, bool symmetric = true)
    -> std::decay_t<std::invoke_result_t<Distance &, int, int>> {
    using Cost = std::decay_t<std::invoke_result_t<Distance &, int, int>>;
    const auto n = tour.size(), l = move.first, r = move.last;
    if (r < l)
        throw std::out_of_range("TSP reverse interval");
    tsp_detail::interval(n, l, r - l);
    if (r - l <= 1)
        return Cost(0);
    const auto next_vertex = [&](std::size_t i) { return tour[i >= l && i < r ? l + r - 1 - i : i]; };
    Cost result = 0;
    const auto change = [&](std::size_t i) {
        const auto j = (i + 1) % n;
        result = tsp_detail::add(
            result, tsp_detail::sub(distance(next_vertex(i), next_vertex(j)), distance(tour[i], tour[j])));
    };
    if (r - l == n) {
        if (!symmetric)
            for (std::size_t i = 0; i < n; ++i)
                change(i);
        return result;
    }
    change((l + n - 1) % n);
    change(r - 1);
    if (!symmetric)
        for (std::size_t i = l; i + 1 < r; ++i)
            change(i);
    return result;
}
struct TspOrOpt {
    std::size_t first, count, after;
}; // move a non-wrapping block after an original index
namespace tsp_detail {
inline bool or_opt_check(std::size_t n, TspOrOpt m) {
    interval(n, m.first, m.count);
    if (!m.count || m.count == n || m.after >= n || (m.after >= m.first && m.after < m.first + m.count))
        throw std::invalid_argument("invalid Or-opt destination/block");
    return m.after == (m.first + n - 1) % n;
}
} // namespace tsp_detail
inline void apply_or_opt(std::vector<int> &tour, TspOrOpt m) {
    if (tsp_detail::or_opt_check(tour.size(), m))
        return;
    if (m.after < m.first)
        std::rotate(tour.begin() + m.after + 1, tour.begin() + m.first, tour.begin() + m.first + m.count);
    else
        std::rotate(tour.begin() + m.first, tour.begin() + m.first + m.count, tour.begin() + m.after + 1);
}
inline void undo_or_opt(std::vector<int> &tour, TspOrOpt m) noexcept {
    if (m.after == (m.first + tour.size() - 1) % tour.size())
        return;
    if (m.after < m.first)
        std::rotate(tour.begin() + m.after + 1, tour.begin() + m.after + 1 + m.count,
                    tour.begin() + m.first + m.count);
    else
        std::rotate(tour.begin() + m.first, tour.begin() + m.after - m.count + 1, tour.begin() + m.after + 1);
}
template <class Distance>
auto or_opt_delta(const std::vector<int> &tour, TspOrOpt m, Distance distance)
    -> std::decay_t<std::invoke_result_t<Distance &, int, int>> {
    using Cost = std::decay_t<std::invoke_result_t<Distance &, int, int>>;
    const auto n = tour.size();
    if (tsp_detail::or_opt_check(n, m))
        return Cost(0);
    const int a = tour[(m.first + n - 1) % n], b = tour[m.first], c = tour[m.first + m.count - 1],
              d = tour[(m.first + m.count) % n], e = tour[m.after], f = tour[(m.after + 1) % n];
    Cost out = tsp_detail::sub(distance(a, d), distance(a, b));
    out = tsp_detail::add(out, tsp_detail::sub(distance(e, b), distance(e, f)));
    return tsp_detail::add(out, tsp_detail::sub(distance(c, f), distance(c, d)));
}
} // namespace cp
