#pragma once
#include <cassert>
#include <limits>
#include <utility>
#include <vector>

// Double-ended monoid aggregation, preserving left-to-right order.
// push/pop: amortized O(1), worst O(n). front/back/prod: O(1).
// Empty prod() is e(); pop/front/back require a nonempty deque.
// Docs: docs/static_query/swag_deque.md
namespace cp {

template <class S, auto op, auto e> class SWAGDeque {
  public:
    int size() const { return int(front_.size() + back_.size()); }
    bool empty() const { return front_.empty() && back_.empty(); }
    void push_front(S value) {
        assert(size() < std::numeric_limits<int>::max());
        append(front_, std::move(value), true);
    }
    void push_back(S value) {
        assert(size() < std::numeric_limits<int>::max());
        append(back_, std::move(value), false);
    }
    void pop_front() {
        assert(!empty());
        if (front_.empty()) rebuild(size() / 2 + size() % 2);
        front_.pop_back();
    }
    void pop_back() {
        assert(!empty());
        if (back_.empty()) rebuild(size() / 2);
        back_.pop_back();
    }
    const S& front() const {
        assert(!empty());
        return front_.empty() ? back_.front().value : front_.back().value;
    }
    const S& back() const {
        assert(!empty());
        return back_.empty() ? front_.front().value : back_.back().value;
    }
    S prod() const {
        if (front_.empty()) return back_.empty() ? e() : back_.back().aggregate;
        if (back_.empty()) return front_.back().aggregate;
        return op(front_.back().aggregate, back_.back().aggregate);
    }
    void clear() { front_.clear(); back_.clear(); }

  private:
    struct Node { S value, aggregate; };
    std::vector<Node> front_, back_;
    static void append(std::vector<Node>& stack, S value, bool left) {
        S aggregate = stack.empty() ? value : left ? op(value, stack.back().aggregate)
                                                  : op(stack.back().aggregate, value);
        stack.push_back({std::move(value), std::move(aggregate)});
    }
    void rebuild(int left_size) {
        const int n = size(), old_left = int(front_.size());
        const auto at = [&](int i) -> const S& {
            return i < old_left ? front_[old_left - 1 - i].value : back_[i - old_left].value;
        };
        // Build both halves before replacing the old stacks, preserving their
        // contents if allocation, copying, or op throws during rebuilding.
        std::vector<Node> left, right;
        left.reserve(left_size); right.reserve(n - left_size);
        for (int i = left_size; i > 0; --i) append(left, at(i - 1), true);
        for (int i = left_size; i < n; ++i) append(right, at(i), false);
        front_.swap(left); back_.swap(right);
    }
};

}  // namespace cp
