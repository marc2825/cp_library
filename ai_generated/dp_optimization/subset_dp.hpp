#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>
#include "detail/arithmetic.hpp"
#include "../set_function/submask_enumeration.hpp"
#include "../set_function/subset_zeta_transform.hpp"
#include "../set_function/subset_mobius_transform.hpp"

namespace cp {
// 列挙・集合変換は3.19の共通実装へ委譲。既存のincludeと関数名を保持。
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
