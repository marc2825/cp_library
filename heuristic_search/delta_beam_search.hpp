#pragma once
#include "beam_search.hpp"
#include "detail/reversible_path.hpp"
#include "incremental_score.hpp"
#include <optional>
namespace cp {
template <class State, class Move, class Score> struct DeltaBeamResult {
    State best_state;
    Score best_score;
    std::vector<Move> moves;
    std::size_t depth = 0, evaluations = 1, stored_nodes = 1;
    SearchTermination termination = SearchTermination::depth_limit;
};
// One mutable root state, DFS over shared prefixes, apply/undo for each candidate.
// Input state is restored on success, stop, and exception. Apply needs a strong guarantee.
template <class State, class Evaluate, class Moves, class Delta, class Apply, class Undo>
auto delta_beam_search(State &state, Evaluate evaluate, Moves enumerate, Delta delta, Apply apply, Undo undo,
                       const BeamSearchOptions &options = {}) {
    using Score = heuristic_detail::ScoreOf<Evaluate, State>;
    using Move = typename std::decay_t<std::invoke_result_t<Moves &, const State &>>::value_type;
    heuristic_detail::validate_goal(options.goal);
    if (!options.width)
        throw std::invalid_argument("beam width must be positive");
    const auto initial_score = evaluate(std::as_const(state));
    heuristic_detail::validate_score(initial_score);
    struct Node {
        std::size_t parent;
        std::optional<Move> move;
        Score score;
        std::vector<std::size_t> children;
    };
    std::vector<Node> nodes{{0, std::nullopt, initial_score, {}}};
    std::vector<std::size_t> frontier{0};
    std::size_t depth = 0, evaluations = 1;
    SearchTermination termination = SearchTermination::depth_limit;
    for (; depth < options.max_depth;) {
        std::vector<bool> active(nodes.size());
        std::vector<std::size_t> rank(nodes.size(), frontier.size());
        for (std::size_t i = 0; i < frontier.size(); ++i) {
            auto q = frontier[i];
            rank[q] = i;
            while (!active[q]) {
                active[q] = true;
                if (!q)
                    break;
                q = nodes[q].parent;
            }
        }
        struct Candidate {
            std::size_t parent, rank, child;
            Move move;
            Score score;
        };
        std::vector<Candidate> candidates;
        bool stopped = false;
        {
            heuristic_detail::ReversiblePath<State, Move, Apply, Undo> path(state, apply, undo);
            struct Frame {
                std::size_t node, next;
            };
            std::vector<Frame> stack{{0, 0}};
            while (!stack.empty()) {
                if (options.stop && options.stop()) {
                    stopped = true;
                    break;
                }
                auto &top = stack.back();
                const auto q = top.node;
                if (rank[q] != frontier.size()) {
                    auto moves = enumerate(std::as_const(state));
                    std::size_t child = 0;
                    Score score = nodes[q].score;
                    for (const auto &move : moves) {
                        if (options.stop && options.stop()) {
                            stopped = true;
                            break;
                        }
                        heuristic_detail::count_evaluation(evaluations);
                        try_scored_move(state, score, move, delta, apply, undo,
                                        [&](const State &, Score, Score value) {
                                            candidates.push_back({q, rank[q], child, move, value});
                                            return false;
                                        });
                        ++child;
                    }
                    if (stopped)
                        break;
                    stack.pop_back();
                    if (q)
                        path.pop();
                    continue;
                }
                while (top.next < nodes[q].children.size() && !active[nodes[q].children[top.next]])
                    ++top.next;
                if (top.next == nodes[q].children.size()) {
                    stack.pop_back();
                    if (q)
                        path.pop();
                } else {
                    const auto child = nodes[q].children[top.next++];
                    path.push(*nodes[child].move);
                    stack.push_back({child, 0});
                }
            }
        }
        if (stopped) {
            termination = SearchTermination::stopped;
            break;
        }
        if (candidates.empty()) {
            termination = SearchTermination::exhausted;
            break;
        }
        std::sort(candidates.begin(), candidates.end(), [&](const Candidate &a, const Candidate &b) {
            if (a.score != b.score)
                return heuristic_detail::better(a.score, b.score, options.goal);
            return std::pair{a.rank, a.child} < std::pair{b.rank, b.child};
        });
        if (candidates.size() > options.width)
            candidates.erase(candidates.begin() + options.width, candidates.end());
        frontier.clear();
        for (auto &c : candidates) {
            const auto id = nodes.size();
            nodes.push_back({c.parent, std::move(c.move), c.score, {}});
            nodes[c.parent].children.push_back(id);
            frontier.push_back(id);
        }
        ++depth;
    }
    const auto best = frontier.front();
    std::vector<Move> moves;
    for (auto q = best; q; q = nodes[q].parent)
        moves.push_back(*nodes[q].move);
    std::reverse(moves.begin(), moves.end());
    heuristic_detail::ReversiblePath<State, Move, Apply, Undo> path(state, apply, undo);
    for (const auto &move : moves)
        path.push(move);
    State best_state = state;
    path.rewind();
    return DeltaBeamResult<State, Move, Score>{
        std::move(best_state), nodes[best].score, std::move(moves), depth,
        evaluations,           nodes.size(),      termination};
}
} // namespace cp
