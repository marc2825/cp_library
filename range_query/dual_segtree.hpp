#pragma once
#include <cassert>
#include <vector>

// Range actions / point get or set. 0-based, [l, r).
// mapping(F, S), composition(f, g) = f after g, id(). No F equality required.
// Docs: docs/range_query/dual_segtree.md
namespace cp {

template <class S, class F, auto mapping, auto composition, auto id>
class DualSegtree {
  public:
    DualSegtree() : DualSegtree(0, S{}) {}
    explicit DualSegtree(int n, S initial = S{}) : n_(n) {
        assert(0 <= n && n <= 100000000);
        base_ = 1;
        log_ = 0;
        while (base_ < n) { base_ <<= 1; ++log_; }
        values_.assign(base_, initial);
        lazy_.assign(base_, id());
    }
    explicit DualSegtree(const std::vector<S>& a) : DualSegtree(int(a.size())) {
        for (int i = 0; i < n_; ++i) values_[i] = a[i];
    }
    int size() const { return n_; }
    S get(int p) {
        assert(0 <= p && p < n_);
        thrust(p + base_);
        return values_[p];
    }
    void set(int p, S x) {
        assert(0 <= p && p < n_);
        thrust(p + base_);
        values_[p] = x;
    }
    void apply(int p, F f) {
        assert(0 <= p && p < n_);
        thrust(p + base_);
        values_[p] = mapping(f, values_[p]);
    }
    void apply(int l, int r, F f) {
        assert(0 <= l && l <= r && r <= n_);
        if (l == r) return;
        l += base_;
        r += base_;
        // Older actions must precede f, even for noncommutative updates.
        thrust(l);
        thrust(r - 1);
        while (l < r) {
            if (l & 1) all_apply(l++, f);
            if (r & 1) all_apply(--r, f);
            l >>= 1;
            r >>= 1;
        }
    }

  private:
    int n_, base_, log_;
    std::vector<S> values_;
    std::vector<F> lazy_;
    void all_apply(int k, F f) {
        if (k < base_) lazy_[k] = composition(f, lazy_[k]);
        else values_[k - base_] = mapping(f, values_[k - base_]);
    }
    void push(int k) {
        all_apply(2 * k, lazy_[k]);
        all_apply(2 * k + 1, lazy_[k]);
        lazy_[k] = id();
    }
    void thrust(int k) {
        for (int i = log_; i > 0; --i) push(k >> i);
    }
};

}  // namespace cp
