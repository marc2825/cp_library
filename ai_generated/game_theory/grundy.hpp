#pragma once
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
#include "../graph/graph.hpp"

namespace cp {
// Nonnegative values; large labels are ignored without allocating by their magnitude.
inline int grundy_mex(const std::vector<int>& values) {
    if (values.size() > std::size_t(std::numeric_limits<int>::max()))
        throw std::length_error("too many mex values");
    std::vector<bool> seen(values.size() + 1, false);
    for (int x : values) {
        if (x < 0) throw std::invalid_argument("Grundy values must be nonnegative");
        if (std::size_t(x) < seen.size()) seen[x] = true;
    }
    std::size_t answer = 0;
    while (seen[answer]) ++answer;
    return static_cast<int>(answer);
}
struct GrundyResult {
    std::vector<int> values;
    std::vector<int> topological_order; // A move always goes forward in this order.
};
// Normal-play impartial finite game. Every component must be a DAG, including unreachable ones.
inline GrundyResult calculate_grundy(const std::vector<std::vector<int>>& moves) {
    if (moves.size() > std::size_t(std::numeric_limits<int>::max()))
        throw std::length_error("too many game positions");
    const int n = static_cast<int>(moves.size());
    std::vector<std::size_t> indegree(moves.size(), 0);
    for (const auto& row : moves) for (int to : row) {
        if (to < 0 || to >= n) throw std::out_of_range("game transition endpoint");
        if (indegree[to] == std::numeric_limits<std::size_t>::max())
            throw std::length_error("too many game transitions");
        ++indegree[to];
    }
    GrundyResult result;
    auto& order = result.topological_order; order.reserve(moves.size());
    for (int v = 0; v < n; ++v) if (!indegree[v]) order.push_back(v);
    for (std::size_t head = 0; head < order.size(); ++head)
        for (int to : moves[order[head]]) if (--indegree[to] == 0) order.push_back(to);
    if (order.size() != moves.size()) throw std::invalid_argument("game graph must be acyclic");
    result.values.assign(moves.size(), 0);
    std::vector<int> seen(moves.size(), -1);
    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        const int v = *it;
        for (int to : moves[v]) seen[result.values[to]] = v;
        int value = 0;
        while (value < n && seen[value] == v) ++value;
        result.values[v] = value;
    }
    return result;
}
namespace game_detail {
template<class Cost> std::vector<std::vector<int>> game_transitions(const Graph<Cost>& graph) {
    if (!graph.directed()) throw std::invalid_argument("game graph must be directed");
    std::vector<std::vector<int>> moves(graph.size());
    for (int v = 0; v < graph.size(); ++v) {
        moves[v].reserve(graph[v].size());
        for (const auto& edge : graph[v]) moves[v].push_back(edge.to);
    }
    return moves;
}
} // namespace game_detail
template<class Cost> GrundyResult calculate_grundy(const Graph<Cost>& graph) {
    return calculate_grundy(game_detail::game_transitions(graph));
}
} // namespace cp
