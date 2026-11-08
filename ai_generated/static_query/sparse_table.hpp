#pragma once
#include <cassert>
#include <utility>
#include <vector>

// Immutable range product of an idempotent monoid. 0-based, [l, r).
// op must be associative and satisfy op(x,x)==x. Empty product is e().
// Docs: docs/static_query/sparse_table.md
namespace cp {

template <class S, auto op, auto e> class SparseTable {
  public:
    SparseTable() = default;
    explicit SparseTable(const std::vector<S>& a) { build(a); }
    void build(const std::vector<S>& a) {
        assert(a.size() <= 100000000);
        n_ = int(a.size());
        logs_.assign(n_ + 1, 0);
        for (int i = 2; i <= n_; ++i) logs_[i] = logs_[i / 2] + 1;
        table_.clear();
        if (n_ == 0) return;
        table_.push_back(a);
        for (int k = 1; k <= logs_[n_]; ++k) {
            int width = 1 << k, half = width / 2;
            std::vector<S> row;
            row.reserve(n_ - width + 1);
            for (int i = 0; i <= n_ - width; ++i)
                row.push_back(op(table_[k - 1][i], table_[k - 1][i + half]));
            table_.push_back(std::move(row));
        }
    }
    int size() const { return n_; }
    bool empty() const { return n_ == 0; }
    S get(int p) const {
        assert(0 <= p && p < n_);
        return table_[0][p];
    }
    S prod(int l, int r) const {
        assert(0 <= l && l <= r && r <= n_);
        if (l == r) return e();
        int k = logs_[r - l];
        return op(table_[k][l], table_[k][r - (1 << k)]);
    }
    S all_prod() const { return prod(0, n_); }

  private:
    int n_ = 0;
    std::vector<int> logs_;
    std::vector<std::vector<S>> table_;
};

}  // namespace cp
