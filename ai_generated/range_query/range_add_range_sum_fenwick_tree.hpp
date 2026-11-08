#pragma once
#include <cassert>
#include <vector>
#include "fenwick_tree.hpp"

// Range add / range sum with two BITs. 0-based, [l, r).
// T needs T(0), +=, subtraction, and T * int (distributive scaling).
// Docs: docs/range_query/range_add_range_sum_fenwick_tree.md
namespace cp {

template <class T> class RangeAddRangeSumFenwickTree {
  public:
    RangeAddRangeSumFenwickTree() : RangeAddRangeSumFenwickTree(0) {}
    explicit RangeAddRangeSumFenwickTree(int n) : slope_(n), offset_(n) {}
    explicit RangeAddRangeSumFenwickTree(const std::vector<T>& a)
        : slope_(differences(a, false)), offset_(differences(a, true)) {}
    int size() const { return slope_.size(); }
    void add(int l, int r, T x) {
        assert(0 <= l && l <= r && r <= size());
        if (l == r) return;
        slope_.add(l, x);
        offset_.add(l, x * l);
        if (r < size()) {
            slope_.add(r, T(0) - x);
            offset_.add(r, (T(0) - x) * r);
        }
    }
    T sum(int r) const {
        assert(0 <= r && r <= size());
        return slope_.sum(r) * r - offset_.sum(r);
    }
    T sum(int l, int r) const {
        assert(0 <= l && l <= r && r <= size());
        return sum(r) - sum(l);
    }
    T get(int p) const {
        assert(0 <= p && p < size());
        return sum(p, p + 1);
    }
    void set(int p, T x) { add(p, p + 1, x - get(p)); }

  private:
    FenwickTree<T> slope_, offset_;
    static std::vector<T> differences(const std::vector<T>& a, bool weighted) {
        std::vector<T> d(a.size(), T(0));
        for (int i = 0; i < int(a.size()); ++i) {
            d[i] = a[i] - (i == 0 ? T(0) : a[i - 1]);
            if (weighted) d[i] = d[i] * i;
        }
        return d;
    }
};

}  // namespace cp
