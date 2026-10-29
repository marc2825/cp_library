#pragma once
#include <type_traits>
#include <vector>
namespace cp::heuristic_detail {
// Applied edges are always unwound, including allocation/callback exceptions.
template <class State, class Move, class Apply, class Undo> class ReversiblePath {
    State &state_;
    Apply &apply_;
    Undo &undo_;
    std::vector<const Move *> path_;

  public:
    ReversiblePath(State &state, Apply &apply, Undo &undo) : state_(state), apply_(apply), undo_(undo) {
        static_assert(std::is_nothrow_invocable_v<Undo &, State &, const Move &>,
                      "path undo must be noexcept");
    }
    ReversiblePath(const ReversiblePath &) = delete;
    ReversiblePath &operator=(const ReversiblePath &) = delete;
    void push(const Move &move) {
        path_.push_back(&move);
        try {
            apply_(state_, move);
        } catch (...) {
            path_.pop_back();
            throw;
        }
    }
    void pop() noexcept {
        undo_(state_, *path_.back());
        path_.pop_back();
    }
    void rewind() noexcept {
        while (!path_.empty())
            pop();
    }
    ~ReversiblePath() { rewind(); }
};
} // namespace cp::heuristic_detail
