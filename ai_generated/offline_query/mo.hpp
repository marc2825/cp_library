#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>
#include <utility>
#include <vector>

// Static range queries [l,r), including empty ranges. IDs follow insertion order.
// run starts at [0,0): caller must initialize its own accumulator before each run.
// Docs: docs/offline_query/mo.md
namespace cp {

class Mo {
  public:
    explicit Mo(int n) : n_(n) { assert(0 <= n); }
    int add_query(int l, int r) {
        assert(0 <= l && l <= r && r <= n_);
        int id = query_count();
        queries_.emplace_back(l, r);
        return id;
    }
    int size() const { return n_; }
    int query_count() const { return int(queries_.size()); }
    std::vector<int> query_order(int block_size = 0) const {
        assert(0 <= block_size);
        if (block_size == 0)
            block_size = std::max(1, int(n_ / std::sqrt(std::max(1, query_count()))));
        std::vector<int> order(query_count());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int i, int j) {
            int a = queries_[i].first / block_size, b = queries_[j].first / block_size;
            if (a != b) return a < b;
            int x = queries_[i].second, y = queries_[j].second;
            if (x != y) return (a & 1) ? x > y : x < y;
            return i < j;
        });
        return order;
    }
    template <class AddLeft, class AddRight, class RemoveLeft, class RemoveRight, class Answer>
    void run(AddLeft add_left, AddRight add_right, RemoveLeft remove_left,
             RemoveRight remove_right, Answer answer, int block_size = 0) const {
        int l = 0, r = 0;
        for (int id : query_order(block_size)) {
            auto [ql, qr] = queries_[id];
            while (l > ql) add_left(--l);
            while (r < qr) add_right(r++);
            while (l < ql) remove_left(l++);
            while (r > qr) remove_right(--r);
            answer(id);
        }
    }
    template <class Add, class Remove, class Answer>
    void run(Add add, Remove remove, Answer answer, int block_size = 0) const {
        run(add, add, remove, remove, answer, block_size);
    }

  private:
    int n_;
    std::vector<std::pair<int, int>> queries_;
};

}  // namespace cp
