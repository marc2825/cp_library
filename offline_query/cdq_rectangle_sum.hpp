#pragma once
#include <algorithm>
#include <cassert>
#include <numeric>
#include <utility>
#include <vector>
#include "rectangle.hpp"
#include "../ordered_structure/coordinate_compression.hpp"
#include "../range_query/fenwick_tree.hpp"

// Offline point additions / rectangle sums. A query sees only earlier additions.
// solve() is repeatable and preserves recorded operations. Negative weights are allowed.
// Docs: docs/offline_query/cdq_rectangle_sum.md
namespace cp {

template <class Coord = long long, class Weight = long long> class CDQRectangleSum {
  public:
    void add_point(Coord x, Coord y, Weight weight) {
        assert(events_.size() < 100000000);
        events_.push_back({x, y, weight, -1, false});
    }
    int add_query(Coord left, Coord down, Coord right, Coord up) {
        return add_query(Rectangle<Coord>{left, down, right, up});
    }
    int add_query(const Rectangle<Coord>& q) {
        assert(query_count_ < 100000000);
        int id = query_count_++;
        if (!q.empty()) {
            assert(events_.size() <= 99999996);
            events_.push_back({q.right, q.up, Weight(0), id, true});
            events_.push_back({q.left, q.up, Weight(0), id, false});
            events_.push_back({q.right, q.down, Weight(0), id, false});
            events_.push_back({q.left, q.down, Weight(0), id, true});
        }
        return id;
    }
    int query_count() const { return query_count_; }
    void clear() { events_.clear(); query_count_ = 0; }
    std::vector<Weight> solve() const {
        std::vector<Weight> answer(query_count_, Weight(0));
        if (query_count_ == 0 || events_.empty()) return answer;
        std::vector<Coord> ys;
        for (const auto& event : events_)
            if (event.id < 0) ys.push_back(event.y);
        if (ys.empty()) return answer;
        CoordinateCompression<Coord> y(std::move(ys));
        FenwickTree<Weight> bit(y.size());
        int n = int(events_.size());
        std::vector<int> rank(n), order(n), buffer(n);
        for (int i = 0; i < n; ++i) rank[i] = y.lower_bound(events_[i].y);
        std::iota(order.begin(), order.end(), 0);
        auto less_x = [&](int i, int j) { return events_[i].x < events_[j].x; };
        auto cdq = [&](auto&& self, int l, int r) -> void {
            if (r - l <= 1) return;
            int m = l + (r - l) / 2;
            self(self, l, m);
            self(self, m, r);
            // Each half still contains its original time interval, sorted by x.
            int p = l;
            for (int j = m; j < r; ++j) {
                int q = order[j];
                while (p < m && less_x(order[p], q)) {
                    int i = order[p++];
                    if (events_[i].id < 0) bit.add(rank[i], events_[i].weight);
                }
                if (events_[q].id >= 0) {
                    Weight sum = bit.sum(rank[q]);
                    answer[events_[q].id] += events_[q].positive ? sum : Weight(0) - sum;
                }
            }
            // Roll back only inserted updates; a full BIT reset at each node is too slow.
            for (int j = l; j < p; ++j) {
                int i = order[j];
                if (events_[i].id < 0) bit.add(rank[i], Weight(0) - events_[i].weight);
            }
            std::merge(order.begin() + l, order.begin() + m, order.begin() + m,
                       order.begin() + r, buffer.begin() + l, less_x);
            std::copy(buffer.begin() + l, buffer.begin() + r, order.begin() + l);
        };
        cdq(cdq, 0, n);
        return answer;
    }

  private:
    struct Event { Coord x, y; Weight weight; int id; bool positive; };
    std::vector<Event> events_;
    int query_count_ = 0;
};

}  // namespace cp
