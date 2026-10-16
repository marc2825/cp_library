#pragma once
#include <cassert>
#include <vector>

// Point add / range sum. 0-based, [l, r). Additive commutative group T.
// lower_bound(w): first element index i with sum(i+1) >= w; n if absent.
// Search requires nonnegative elements and an ordered, exact additive type.
// Docs: docs/range_query/fenwick_tree.md
namespace cp {

template <class T> class FenwickTree {
  public:
    FenwickTree() : FenwickTree(0) {}
    explicit FenwickTree(int n) : n_(n) {
        assert(0 <= n && n <= 100000000);
        bit_.assign(n + 1, T(0));
    }
    explicit FenwickTree(const std::vector<T>& a) : FenwickTree(int(a.size())) {
        for (int i = 1; i <= n_; ++i) {
            bit_[i] += a[i - 1];
            int parent = i + (i & -i);
            if (parent <= n_) bit_[parent] += bit_[i];
        }
    }
    int size() const { return n_; }
    void add(int p, T x) {
        assert(0 <= p && p < n_);
        for (int i = p + 1; i <= n_; i += i & -i) bit_[i] += x;
    }
    T sum(int r) const {
        assert(0 <= r && r <= n_);
        T result = T(0);
        for (int i = r; i > 0; i -= i & -i) result += bit_[i];
        return result;
    }
    T sum(int l, int r) const {
        assert(0 <= l && l <= r && r <= n_);
        return sum(r) - sum(l);
    }
    T get(int p) const {
        assert(0 <= p && p < n_);
        return sum(p, p + 1);
    }
    void set(int p, T x) { add(p, x - get(p)); }
    int lower_bound(T w) const {
        if (!(T(0) < w)) return 0;
        int p = 0, step = 1;
        while (step <= n_ / 2) step <<= 1;
        for (; step > 0; step >>= 1) {
            int next = p + step;
            if (next <= n_ && bit_[next] < w) {
                w = w - bit_[next];
                p = next;
            }
        }
        return p;
    }

  private:
    int n_;
    std::vector<T> bit_;
};

}  // namespace cp
