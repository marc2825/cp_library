#pragma once
#include <algorithm>
#include <functional>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>
#include "detail/arithmetic.hpp"
#include "state_search_common.hpp"

namespace cp {
// 非負整数辺。heuristicは非負・許容的・goalで0が前提。非整合でも再展開する。
// neighbors(state)はStateSearchEdge<State>の列。有限な到達状態空間を想定。
template<class State, class Neighbors, class Goal, class Heuristic,
         class Hash = std::hash<State>, class Equal = std::equal_to<State>>
std::optional<StateSearchResult<State>> a_star(const State& start, Neighbors neighbors,
        Goal goal, Heuristic heuristic, Hash hash = {}, Equal equal = {}) {
    struct Node { State state; long long g; std::size_t parent; int action; };
    struct Entry { long long f; std::size_t node; bool operator>(const Entry& b) const {
        return f != b.f ? f > b.f : node > b.node;
    }};
    auto estimate = [&](const State& state) {
        const long long h = heuristic(state);
        if (h < 0) throw std::invalid_argument("negative A* heuristic");
        return h;
    };
    std::vector<Node> nodes{{start, 0, 0, -1}};
    std::unordered_map<State, std::size_t, Hash, Equal> best(0, hash, equal);
    best.emplace(start, 0);
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    queue.push({estimate(start), 0});
    std::size_t expanded = 0;
    while (!queue.empty()) {
        const auto entry = queue.top(); queue.pop();
        const Node current = nodes[entry.node];
        if (best.at(current.state) != entry.node) continue;
        if (goal(current.state)) {
            if (estimate(current.state) != 0) throw std::invalid_argument("A* goal heuristic must be zero");
            StateSearchResult<State> result{current.g, {}, {}, expanded};
            for (auto id = entry.node;; id = nodes[id].parent) {
                result.states.push_back(nodes[id].state);
                if (id == 0) break;
                result.actions.push_back(nodes[id].action);
            }
            std::reverse(result.states.begin(), result.states.end());
            std::reverse(result.actions.begin(), result.actions.end());
            return result;
        }
        ++expanded;
        for (const auto& edge : neighbors(current.state)) {
            if (edge.cost < 0) throw std::invalid_argument("negative A* edge");
            const auto g = detail::dp_add(current.g, edge.cost);
            auto it = best.find(edge.to);
            if (it != best.end() && nodes[it->second].g <= g) continue;
            const auto f = detail::dp_add(g, estimate(edge.to));
            const auto id = nodes.size();
            nodes.push_back({edge.to, g, entry.node, edge.action});
            best.insert_or_assign(edge.to, id);
            queue.push({f, id});
        }
    }
    return std::nullopt;
}
} // namespace cp
