#pragma once
#include <algorithm>
#include <functional>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include "state_search_common.hpp"

namespace cp {
// 単位費用の有向状態空間。reverse(v)は元のu->vを{u,1,action}として返す。
// 小さい側のfrontierを層ごと展開。actionは復元後も元の向き。
template<class State, class Forward, class Reverse,
         class Hash = std::hash<State>, class Equal = std::equal_to<State>>
std::optional<StateSearchResult<State>> bidirectional_bfs(const State& start, const State& target,
        Forward forward, Reverse reverse, Hash hash = {}, Equal equal = {}) {
    struct Node { State state; std::size_t parent; int action; long long depth; };
    std::vector<Node> nodes[2];
    nodes[0].push_back({start, 0, -1, 0}); nodes[1].push_back({target, 0, -1, 0});
    using Map = std::unordered_map<State, std::size_t, Hash, Equal>;
    Map seen[2] = {Map(0, hash, equal), Map(0, hash, equal)};
    seen[0].emplace(start, 0); seen[1].emplace(target, 0);
    std::vector<std::size_t> frontier[2] = {{0}, {0}};
    std::optional<std::pair<std::size_t, std::size_t>> meet;
    if (equal(start, target)) meet = {0, 0};
    std::size_t expanded = 0;
    while (!meet && !frontier[0].empty() && !frontier[1].empty()) {
        const int side = frontier[0].size() <= frontier[1].size() ? 0 : 1;
        std::vector<std::size_t> next;
        long long best = 0;
        auto visit = [&](std::size_t id, const auto& edges) {
            const auto depth = nodes[side][id].depth;
            for (const auto& edge : edges) {
                if (edge.cost != 1) throw std::invalid_argument("bidirectional BFS requires unit costs");
                auto inserted = seen[side].emplace(edge.to, nodes[side].size());
                if (!inserted.second) continue;
                const auto child = nodes[side].size();
                nodes[side].push_back({edge.to, id, edge.action, depth + 1});
                next.push_back(child);
                auto other = seen[1-side].find(edge.to);
                if (other != seen[1-side].end()) {
                    const auto distance = depth + 1 + nodes[1-side][other->second].depth;
                    if (!meet || distance < best) {
                        best = distance;
                        meet = side == 0 ? std::make_pair(child, other->second) : std::make_pair(other->second, child);
                    }
                }
            }
        };
        for (auto id : frontier[side]) {
            ++expanded;
            // コピーによりnodesの再確保と利用者の参照返却を切り離す。
            const State state = nodes[side][id].state;
            if (side == 0) visit(id, forward(state)); else visit(id, reverse(state));
        }
        frontier[side] = std::move(next);
    }
    if (!meet) return std::nullopt;
    StateSearchResult<State> result{nodes[0][meet->first].depth + nodes[1][meet->second].depth, {}, {}, expanded};
    for (auto id = meet->first;; id = nodes[0][id].parent) {
        result.states.push_back(nodes[0][id].state);
        if (id == 0) break;
        result.actions.push_back(nodes[0][id].action);
    }
    std::reverse(result.states.begin(), result.states.end());
    std::reverse(result.actions.begin(), result.actions.end());
    for (auto id = meet->second; id != 0;) {
        result.actions.push_back(nodes[1][id].action);
        id = nodes[1][id].parent;
        result.states.push_back(nodes[1][id].state);
    }
    return result;
}
} // namespace cp
