#pragma once
#include <algorithm>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
#include "../basic_utility/big_integer.hpp"
#include "detail/arithmetic.hpp"

namespace cp {
struct FrontierConnectivityState {
    std::vector<int> degree, component; // component=-1は未使用
    bool finished = false;
    void normalize() {
        std::map<int, int> labels;
        for (auto& c : component) if (c >= 0) {
            auto it = labels.emplace(c, static_cast<int>(labels.size())).first;
            c = it->second;
        }
    }
    bool operator<(const FrontierConnectivityState& other) const {
        return std::tie(degree, component, finished) < std::tie(other.degree, other.component, other.finished);
    }
};
// 辺IDを区別する無向loopなしグラフ。指定次数0..2、使用頂点が連結な辺部分集合を数える。
// 頂点は最後の接続辺の処理後に忘れる。辺順序は利用者が指定する。
template<class Count = BigInt>
Count frontier_connected_degree_count(int n, const std::vector<std::pair<int,int>>& edges,
                                      const std::vector<int>& required_degree) {
    if (n < 0 || required_degree.size() != static_cast<std::size_t>(n))
        throw std::invalid_argument("frontier dimensions");
    std::vector<std::size_t> last(n, edges.size());
    for (std::size_t i = 0; i < edges.size(); ++i) {
        auto [u,v] = edges[i];
        if (u < 0 || u >= n || v < 0 || v >= n || u == v) throw std::invalid_argument("frontier edge");
        last[u] = last[v] = i;
    }
    bool any = false;
    for (int v = 0; v < n; ++v) {
        if (required_degree[v] < 0 || required_degree[v] > 2) throw std::invalid_argument("frontier degree");
        any = any || required_degree[v];
    }
    for (int v = 0; v < n; ++v) if (required_degree[v] && last[v] == edges.size()) return Count(0);
    if (!any) return Count(1); // 空部分グラフを1個とする。
    std::map<FrontierConnectivityState, Count> dp;
    dp[{}] = Count(1);
    std::vector<int> frontier;
    for (std::size_t e = 0; e < edges.size(); ++e) {
        const auto [u,v] = edges[e];
        auto active = frontier;
        for (int x : {u,v}) if (std::find(active.begin(), active.end(), x) == active.end()) active.push_back(x);
        const int a = static_cast<int>(std::find(active.begin(), active.end(), u) - active.begin());
        const int b = static_cast<int>(std::find(active.begin(), active.end(), v) - active.begin());
        std::map<FrontierConnectivityState, Count> next;
        for (const auto& entry : dp) for (int take = 0; take < 2; ++take) {
            auto state = entry.first;
            state.degree.resize(active.size(), 0); state.component.resize(active.size(), -1);
            if (take) {
                if (state.finished || state.degree[a] == required_degree[u] || state.degree[b] == required_degree[v]) continue;
                ++state.degree[a]; ++state.degree[b];
                int ca = state.component[a], cb = state.component[b];
                if (ca < 0 && cb < 0) ca = static_cast<int>(active.size());
                if (ca < 0) ca = cb;
                if (cb >= 0 && ca != cb) for (auto& c : state.component) if (c == cb) c = ca;
                state.component[a] = state.component[b] = ca;
            }
            FrontierConnectivityState reduced;
            reduced.finished = state.finished;
            bool valid = true;
            std::map<int, bool> remaining;
            for (std::size_t j = 0; j < active.size(); ++j) {
                const auto c = state.component[j];
                if (c >= 0) remaining.emplace(c, false);
                if (last[active[j]] == e) {
                    if (state.degree[j] != required_degree[active[j]]) { valid = false; break; }
                } else {
                    reduced.degree.push_back(state.degree[j]); reduced.component.push_back(c);
                    if (c >= 0) remaining[c] = true;
                }
            }
            if (!valid) continue;
            int closed = 0;
            for (const auto& c : remaining) closed += !c.second;
            if (closed) {
                if (closed != 1 || remaining.size() != 1 || state.finished) continue;
                reduced.finished = true;
            }
            reduced.normalize();
            detail::dp_count_add(next[reduced], entry.second);
        }
        frontier.clear();
        for (int x : active) if (last[x] != e) frontier.push_back(x);
        dp = std::move(next);
    }
    FrontierConnectivityState final; final.finished = true;
    auto found = dp.find(final);
    return found == dp.end() ? Count(0) : found->second;
}
// 全頂点を通る無向Hamiltonian path/cycle。向き・始点の違いで重複計数しない。
template<class Count = BigInt>
Count frontier_hamiltonian_path_count(int n, const std::vector<std::pair<int,int>>& edges, int source, int target) {
    if (n <= 0 || source < 0 || source >= n || target < 0 || target >= n)
        throw std::invalid_argument("frontier path endpoints");
    std::vector<int> degree(n, 2); degree[source] = degree[target] = 1;
    if (source == target) {
        // 共通coreで入力検査を行う。
        frontier_connected_degree_count<Count>(n, edges, std::vector<int>(n));
        return Count(n == 1 ? 1 : 0);
    }
    return frontier_connected_degree_count<Count>(n, edges, degree);
}
template<class Count = BigInt>
Count frontier_hamiltonian_cycle_count(int n, const std::vector<std::pair<int,int>>& edges) {
    if (n < 0) throw std::invalid_argument("frontier vertex count");
    auto count = frontier_connected_degree_count<Count>(n, edges, std::vector<int>(n, 2));
    return n == 0 ? Count(0) : count;
}
} // namespace cp
