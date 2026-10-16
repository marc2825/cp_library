#pragma once
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "grundy.hpp"

namespace cp {
struct DAGGameMove { std::size_t component; int from, to; };
// Each component is an independent token. One move changes exactly one token's vertex.
class DAGGame {
    std::vector<std::vector<int>> moves_;
    GrundyResult result_;
    void check(int v) const {
        if (v < 0 || v >= size()) throw std::out_of_range("DAG game vertex");
    }
public:
    explicit DAGGame(std::vector<std::vector<int>> moves = {})
        : moves_(std::move(moves)), result_(calculate_grundy(moves_)) {}
    template<class Cost> explicit DAGGame(const Graph<Cost>& graph)
        : DAGGame(game_detail::game_transitions(graph)) {}
    int size() const { return static_cast<int>(moves_.size()); }
    const std::vector<std::vector<int>>& transitions() const { return moves_; }
    const std::vector<int>& grundy() const { return result_.values; }
    const std::vector<int>& topological_order() const { return result_.topological_order; }
    int grundy(int v) const { check(v); return result_.values[v]; }
    bool winning(int v) const { return grundy(v) != 0; }
    std::optional<int> move_to_grundy(int v, int target) const {
        check(v);
        if (target < 0) throw std::invalid_argument("negative target Grundy value");
        for (int to : moves_[v]) if (result_.values[to] == target) return to;
        return std::nullopt;
    }
    std::optional<int> winning_move(int v) const { return move_to_grundy(v, 0); }
    int nim_sum(const std::vector<int>& positions) const {
        int sum = 0;
        for (int v : positions) sum ^= grundy(v);
        return sum;
    }
    bool winning(const std::vector<int>& positions) const { return nim_sum(positions) != 0; }
    std::optional<DAGGameMove> winning_move(const std::vector<int>& positions) const {
        const int sum = nim_sum(positions);
        if (!sum) return std::nullopt;
        for (std::size_t i = 0; i < positions.size(); ++i) {
            const int from = positions[i], target = sum ^ result_.values[from];
            if (target < result_.values[from]) {
                const auto to = move_to_grundy(from, target);
                if (!to) throw std::logic_error("inconsistent DAG Grundy table");
                return DAGGameMove{i, from, *to};
            }
        }
        return std::nullopt;
    }
};
} // namespace cp
