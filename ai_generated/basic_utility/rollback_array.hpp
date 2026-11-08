#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace cp {
// Fixed-size array; immutable reads ensure every mutation is logged.
template <class T> class RollbackArray {
    static_assert(std::is_copy_constructible_v<T> && std::is_nothrow_swappable_v<T> && std::is_nothrow_destructible_v<T>);
    struct Change { std::size_t index; T old; std::uint64_t serial; };
    std::vector<T> values_;
    std::vector<Change> history_;
    std::uint64_t serial_ = 0, epoch_ = 0;
public:
    class Snapshot {
        friend class RollbackArray;
        const RollbackArray* owner_ = nullptr;
        std::size_t depth_ = 0;
        std::uint64_t tail_ = 0, epoch_ = 0;
    public:
        Snapshot() = default;
    };
    explicit RollbackArray(std::vector<T> values) : values_(std::move(values)) {}
    explicit RollbackArray(std::size_t n, const T& value = T{}) : values_(n, value) {}
    RollbackArray(const RollbackArray&) = delete;
    RollbackArray& operator=(const RollbackArray&) = delete;
    RollbackArray(RollbackArray&&) = delete;
    RollbackArray& operator=(RollbackArray&&) = delete;
    std::size_t size() const { return values_.size(); }
    std::size_t history_size() const { return history_.size(); }
    typename std::vector<T>::const_reference operator[](std::size_t i) const { return values_.at(i); }
    const std::vector<T>& values() const { return values_; }
    void set(std::size_t i, T value) {
        values_.at(i);
        if (serial_ == std::numeric_limits<std::uint64_t>::max()) throw std::length_error("RollbackArray revision limit");
        history_.push_back({i, values_[i], serial_ + 1});
        ++serial_;
        if constexpr (std::is_same_v<T,bool>) values_[i] = value;
        else { using std::swap; swap(values_[i],value); }
    }
    template <class F> void modify(std::size_t i, F&& change) {
        T next = values_.at(i);
        std::invoke(std::forward<F>(change), next);
        set(i,next);
    }
    Snapshot snapshot() const {
        Snapshot s; s.owner_=this; s.depth_=history_.size(); s.tail_=history_.empty()?0:history_.back().serial; s.epoch_=epoch_; return s;
    }
    void rollback(Snapshot s) {
        if (s.owner_ != this || s.epoch_ != epoch_ || s.depth_ > history_.size() ||
            (s.depth_ && history_[s.depth_-1].serial != s.tail_))
            throw std::invalid_argument("snapshot is not on the current history");
        while (history_.size() > s.depth_) {
            auto& change = history_.back();
            if constexpr (std::is_same_v<T,bool>) values_[change.index] = change.old;
            else { using std::swap; swap(values_[change.index], change.old); }
            history_.pop_back();
        }
    }
    void clear_history() {
        if (epoch_ == std::numeric_limits<std::uint64_t>::max()) throw std::length_error("RollbackArray epoch limit");
        history_.clear(); ++epoch_;
    }
};
} // namespace cp
