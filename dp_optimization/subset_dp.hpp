#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
// mask の部分集合を降順に列挙。空集合も既定で含む。詳細: docs/dp_optimization/subset_dp.md。
template <class Callback>
void for_each_submask(std::uint64_t mask, Callback callback, bool include_empty = true) {
    for (auto sub = mask;; sub = (sub - 1) & mask) {
        if (sub || include_empty) callback(sub);
        if (!sub) break;
    }
}
// f[mask] <- sum_{sub subset mask} f[sub]。長さは正の2冪。
template <class T> void subset_zeta_transform(std::vector<T>& f) {
    if (f.empty() || (f.size() & (f.size() - 1))) throw std::invalid_argument("subset table size must be a positive power of two");
    for (std::size_t bit = 1; bit < f.size(); bit <<= 1)
        for (std::size_t mask = 0; mask < f.size(); ++mask) if (mask & bit) f[mask] += f[mask ^ bit];
}
template <class T> void subset_mobius_transform(std::vector<T>& f) {
    if (f.empty() || (f.size() & (f.size() - 1))) throw std::invalid_argument("subset table size must be a positive power of two");
    for (std::size_t bit = 1; bit < f.size(); bit <<= 1)
        for (std::size_t mask = 0; mask < f.size(); ++mask) if (mask & bit) f[mask] -= f[mask ^ bit];
}
struct HamiltonianCycleResult { bool exists = false; long long cost = 0; std::vector<int> vertices; };
// 全頂点を1回ずつ訪れて start に戻る最小巡回路。nullopt は辺なし。20頂点まで。
inline HamiltonianCycleResult minimum_hamiltonian_cycle(
    const std::vector<std::vector<std::optional<long long>>>& cost, int start = 0) {
    const auto n_size = cost.size();
    if (n_size > 20) throw std::length_error("subset TSP supports at most 20 vertices");
    const int n = static_cast<int>(n_size);
    for (const auto& row : cost) if (row.size() != n_size) throw std::invalid_argument("TSP cost matrix must be square");
    if (!n) return {true, 0, {}};
    assert(0 <= start && start < n);
    if (n == 1) return {true, 0, {start}};
    const std::size_t states = std::size_t(1) << n, total = states * n;
    std::vector<long long> dp(total);
    std::vector<unsigned char> reached(total, 0);
    std::vector<signed char> previous(total, -1);
    const std::size_t initial = std::size_t(1) << start;
    reached[initial * n + start] = 1;
    for (std::size_t mask = initial; mask < states; ++mask) {
        if (!(mask & initial)) continue;
        for (int v = 0; v < n; ++v) if (reached[mask * n + v])
            for (int to = 0; to < n; ++to) if (!(mask >> to & 1) && cost[v][to]) {
                const auto next = (mask | (std::size_t(1) << to)) * n + to;
                const auto candidate = detail::dp_add(dp[mask * n + v], *cost[v][to]);
                if (!reached[next] || candidate < dp[next]) {
                    reached[next] = 1; dp[next] = candidate; previous[next] = static_cast<signed char>(v);
                }
            }
    }
    HamiltonianCycleResult result; int end = -1;
    for (int v = 0; v < n; ++v) if (v != start && reached[(states - 1) * n + v] && cost[v][start]) {
        const auto candidate = detail::dp_add(dp[(states - 1) * n + v], *cost[v][start]);
        if (!result.exists || candidate < result.cost) { result.exists = true; result.cost = candidate; end = v; }
    }
    if (!result.exists) return result;
    std::size_t mask = states - 1;
    for (int v = end; v != start;) {
        result.vertices.push_back(v);
        const int p = previous[mask * n + v]; mask ^= std::size_t(1) << v; v = p;
    }
    result.vertices.push_back(start); std::reverse(result.vertices.begin(), result.vertices.end()); result.vertices.push_back(start);
    return result;
}
} // namespace cp
