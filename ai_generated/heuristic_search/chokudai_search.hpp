#pragma once
#include "beam_search.hpp"
#include <optional>
#include <queue>
namespace cp {
enum class ChokudaiTermination { expansion_limit, node_limit, exhausted, stopped };
struct ChokudaiOptions {
    std::size_t max_depth = 100, expansions_per_depth = 1, max_expansions = 10000, max_nodes = 1000000;
    SearchGoal goal = SearchGoal::minimize;
    std::function<bool()> stop;
};
template <class State, class Score> struct ChokudaiResult {
    std::vector<std::optional<ScoredSearchState<State, Score>>> best_at_depth;
    std::size_t deepest = 0, expansions = 0, evaluations = 1, queued = 0;
    ChokudaiTermination termination = ChokudaiTermination::expansion_limit;
    const ScoredSearchState<State, Score> &best() const { return *best_at_depth[deepest]; }
    bool reached_target() const { return deepest + 1 == best_at_depth.size(); }
};
// Priority queues at each depth persist between rounds. Deeper layers need not be exhausted first.
template <class State, class Evaluate, class Expand>
auto chokudai_search(State initial, Evaluate evaluate, Expand expand, const ChokudaiOptions &options = {}) {
    using Score = heuristic_detail::ScoreOf<Evaluate, State>;
    heuristic_detail::validate_goal(options.goal);
    if (!options.expansions_per_depth || !options.max_nodes ||
        options.max_depth == std::numeric_limits<std::size_t>::max())
        throw std::invalid_argument("invalid chokudai budget");
    const auto score = evaluate(std::as_const(initial));
    heuristic_detail::validate_score(score);
    struct Entry {
        State state;
        Score score;
        std::size_t serial;
    };
    const auto compare = [&](const Entry &a, const Entry &b) {
        if (a.score != b.score)
            return heuristic_detail::better(b.score, a.score, options.goal);
        return a.serial > b.serial;
    };
    using Queue = std::priority_queue<Entry, std::vector<Entry>, decltype(compare)>;
    std::vector<Queue> queues;
    for (std::size_t d = 0; d < options.max_depth; ++d)
        queues.emplace_back(compare);
    ChokudaiResult<State, Score> out;
    out.best_at_depth.resize(options.max_depth + 1);
    out.best_at_depth[0] = ScoredSearchState<State, Score>{initial, score};
    if (options.max_depth) {
        queues[0].push({std::move(initial), score, 0});
        out.queued = 1;
    }
    std::size_t serial = 1;
    for (;;) {
        bool progressed = false;
        for (std::size_t depth = 0; depth < options.max_depth; ++depth)
            for (std::size_t k = 0; k < options.expansions_per_depth && !queues[depth].empty(); ++k) {
                if (options.stop && options.stop()) {
                    out.termination = ChokudaiTermination::stopped;
                    return out;
                }
                if (out.expansions == options.max_expansions)
                    return out;
                auto parent = queues[depth].top();
                queues[depth].pop();
                --out.queued;
                ++out.expansions;
                progressed = true;
                auto children = expand(std::as_const(parent.state));
                for (auto &child : children) {
                    if (options.stop && options.stop()) {
                        out.termination = ChokudaiTermination::stopped;
                        return out;
                    }
                    if (out.evaluations == options.max_nodes) {
                        out.termination = ChokudaiTermination::node_limit;
                        return out;
                    }
                    const auto value = evaluate(std::as_const(child));
                    heuristic_detail::validate_score(value);
                    ++out.evaluations;
                    const auto d = depth + 1;
                    if (!out.best_at_depth[d] ||
                        heuristic_detail::better(value, out.best_at_depth[d]->score, options.goal))
                        out.best_at_depth[d] = ScoredSearchState<State, Score>{child, value};
                    out.deepest = std::max(out.deepest, d);
                    if (d < options.max_depth) {
                        queues[d].push({std::move(child), value, serial++});
                        ++out.queued;
                    }
                }
            }
        if (!progressed) {
            out.termination = ChokudaiTermination::exhausted;
            return out;
        }
    }
}
} // namespace cp
