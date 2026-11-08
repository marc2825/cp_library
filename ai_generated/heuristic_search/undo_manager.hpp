#pragma once
#include "../numerical_analysis/common.hpp"
#include "search_common.hpp"
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>
namespace cp {
// Callbacks must restore the pre-state even after a partially failing forward operation.
// Undo is noexcept. Referenced objects must outlive their history entries.
class UndoManager {
    struct Action {
        virtual void undo() noexcept = 0;
        virtual ~Action() = default;
    };
    template <class F> struct ActionModel final : Action {
        F function;
        explicit ActionModel(F f) : function(std::move(f)) {}
        void undo() noexcept override { function(); }
    };
    struct Entry {
        std::unique_ptr<Action> action;
        std::uint64_t serial;
    };
    std::vector<Entry> history_;
    std::vector<std::size_t> guards_;
    std::uint64_t serial_ = 0, epoch_ = 0;
    bool performing_ = false;
    void not_performing() const {
        if (performing_)
            throw std::logic_error("history cannot be changed inside a forward callback");
    }
    void unwind(std::size_t depth) noexcept {
        while (history_.size() > depth) {
            history_.back().action->undo();
            history_.pop_back();
        }
    }

  public:
    class Snapshot {
        friend class UndoManager;
        const UndoManager *owner_ = nullptr;
        std::size_t depth_ = 0;
        std::uint64_t tail_ = 0, epoch_ = 0;

      public:
        Snapshot() = default;
    };
    UndoManager() = default;
    UndoManager(const UndoManager &) = delete;
    UndoManager &operator=(const UndoManager &) = delete;
    UndoManager(UndoManager &&) = delete;
    UndoManager &operator=(UndoManager &&) = delete;
    std::size_t history_size() const { return history_.size(); }
    Snapshot snapshot() const {
        Snapshot s;
        s.owner_ = this;
        s.depth_ = history_.size();
        s.tail_ = history_.empty() ? 0 : history_.back().serial;
        s.epoch_ = epoch_;
        return s;
    }
    template <class Forward, class Backward> void perform(Forward forward, Backward backward) {
        static_assert(std::is_nothrow_invocable_v<Backward &>, "undo must be noexcept");
        not_performing();
        if (serial_ == UINT64_MAX)
            throw std::length_error("undo revision limit");
        auto action = std::make_unique<ActionModel<Backward>>(std::move(backward));
        const auto depth = history_.size();
        history_.push_back({std::move(action), ++serial_});
        // A forward callback mutates its target only; nested logging uses transaction().
        performing_ = true;
        try {
            forward();
        } catch (...) {
            performing_ = false;
            unwind(depth);
            throw;
        }
        performing_ = false;
    }
    template <class T> void set(T &target, T value) {
        static_assert(std::is_nothrow_swappable_v<T>, "logged assignment needs noexcept swap");
        T old = target;
        perform(
            [&] {
                using std::swap;
                swap(target, value);
            },
            [&target, old = std::move(old)]() mutable noexcept {
                using std::swap;
                swap(target, old);
            });
    }
    void rollback(Snapshot s) {
        not_performing();
        if (s.owner_ != this || s.epoch_ != epoch_ || s.depth_ > history_.size() ||
            (s.depth_ && history_[s.depth_ - 1].serial != s.tail_) ||
            (!guards_.empty() && s.depth_ < guards_.back()))
            throw std::invalid_argument("snapshot is not on the current transaction history");
        unwind(s.depth_);
    }
    void undo_last() {
        not_performing();
        if (history_.empty() || (!guards_.empty() && history_.size() == guards_.back()))
            throw std::out_of_range("no undoable operation in this transaction");
        unwind(history_.size() - 1);
    }
    template <class F> void transaction(F function) {
        not_performing();
        const auto depth = history_.size();
        guards_.push_back(depth);
        try {
            function(*this);
        } catch (...) {
            guards_.pop_back();
            unwind(depth);
            throw;
        }
        guards_.pop_back();
    }
    void clear_history() {
        not_performing();
        if (!guards_.empty())
            throw std::logic_error("cannot discard an active transaction");
        if (epoch_ == UINT64_MAX)
            throw std::length_error("undo epoch limit");
        history_.clear();
        ++epoch_;
    }
};
template <class Score> struct ScoreAudit {
    Score incremental, recomputed;
    bool consistent;
};
template <class State, class Score, class Evaluate>
ScoreAudit<Score> audit_score(const State &state, Score score, Evaluate evaluate,
                              numerical::ErrorTolerance tolerance = {}) {
    static_assert(std::is_same_v<Score, heuristic_detail::ScoreOf<Evaluate, State>>, "score type differs");
    heuristic_detail::validate_score(score);
    tolerance.validate();
    const Score actual = evaluate(state);
    heuristic_detail::validate_score(actual);
    bool equal;
    if constexpr (std::is_integral_v<Score>)
        equal = score == actual;
    else
        equal = numerical::almost_equal(score, actual, tolerance);
    return {score, actual, equal};
}
} // namespace cp
