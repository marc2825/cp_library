#pragma once
#include <cassert>
#include <limits>
#include <utility>
#include <vector>

// FIFO aggregation of a monoid; preserves left-to-right order (no inverse needed).
// prod() on an empty queue returns e(). push/pop/front are amortized O(1).
// Docs: docs/static_query/swag.md
namespace cp {

template <class S, auto op, auto e> class SWAG {
  public:
    int size() const { return int(front_.size() + back_.size()); }
    bool empty() const { return front_.empty() && back_.empty(); }
    void push(S value) {
        assert(size() < std::numeric_limits<int>::max());
        S aggregate = back_.empty() ? value : op(back_.back().aggregate, value);
        back_.push_back({std::move(value), std::move(aggregate)});
    }
    void pop() { assert(!empty()); transfer(); front_.pop_back(); }
    const S& front() { assert(!empty()); transfer(); return front_.back().value; }
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
    void transfer() {
        if (!front_.empty()) return;
        while (!back_.empty()) {
            S value = std::move(back_.back().value);
            back_.pop_back();
            S aggregate = front_.empty() ? value : op(value, front_.back().aggregate);
            front_.push_back({std::move(value), std::move(aggregate)});
        }
    }
};

}  // namespace cp
