#pragma once
#include <algorithm>
#include <functional>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include "detail/arithmetic.hpp"
#include "state_search_common.hpp"

namespace cp {
// 非負整数辺・許容的な非負heuristic・goalで0。経路上の循環だけを除去。
// 明示stackで最小の超過f値まで閾値を上げる。有限状態空間なら到達不能も終了。
template<class State, class Neighbors, class Goal, class Heuristic,
         class Hash = std::hash<State>, class Equal = std::equal_to<State>>
std::optional<StateSearchResult<State>> ida_star(const State& start, Neighbors neighbors,
        Goal goal, Heuristic heuristic, Hash hash = {}, Equal equal = {}) {
    auto estimate = [&](const State& s) {
        const long long h = heuristic(s);
        if (h < 0) throw std::invalid_argument("negative IDA* heuristic");
        return h;
    };
    struct Frame { State state; long long g; int action; bool entered = false;
        std::vector<StateSearchEdge<State>> edges; std::size_t next = 0; };
    long long bound = estimate(start);
    std::size_t expanded = 0;
    for (;;) {
        std::optional<long long> next_bound;
        std::vector<Frame> stack{{start, 0, -1, false, {}, 0}};
        std::unordered_set<State, Hash, Equal> path(0, hash, equal);
        path.insert(start);
        auto pop = [&]() { path.erase(stack.back().state); stack.pop_back(); };
        while (!stack.empty()) {
            auto& frame = stack.back();
            if (!frame.entered) {
                const auto h = estimate(frame.state), f = detail::dp_add(frame.g, h);
                if (f > bound) {
                    if (!next_bound || f < *next_bound) next_bound = f;
                    pop(); continue;
                }
                if (goal(frame.state)) {
                    if (h != 0) throw std::invalid_argument("IDA* goal heuristic must be zero");
                    StateSearchResult<State> result{frame.g, {}, {}, expanded};
                    for (const auto& x : stack) {
                        result.states.push_back(x.state);
                        result.actions.push_back(x.action);
                    }
                    result.actions.erase(result.actions.begin());
                    return result;
                }
                ++expanded; frame.entered = true;
                for (const auto& edge : neighbors(frame.state)) {
                    if (edge.cost < 0) throw std::invalid_argument("negative IDA* edge");
                    frame.edges.push_back(edge);
                }
            }
            if (frame.next == frame.edges.size()) { pop(); continue; }
            const auto edge = frame.edges[frame.next++];
            if (path.count(edge.to)) continue;
            const auto g = detail::dp_add(frame.g, edge.cost);
            path.insert(edge.to);
            stack.push_back({edge.to, g, edge.action, false, {}, 0});
        }
        if (!next_bound) return std::nullopt;
        bound = *next_bound;
    }
}
} // namespace cp
