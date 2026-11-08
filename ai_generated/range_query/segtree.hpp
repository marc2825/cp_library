#pragma once
#include <cassert>
#include <vector>

// Point set / ordered range product of a monoid. 0-based, [l, r).
// Docs: docs/range_query/segtree.md
// Adapted from AtCoder Library (CC0): https://github.com/atcoder/ac-library
namespace cp {

template <class S, auto op, auto e> struct Segtree {
  public:
    Segtree() : Segtree(0) {}
    explicit Segtree(int n) : Segtree(initial_values(n)) {}
    explicit Segtree(const std::vector<S>& v) : n_(int(v.size())) {
        assert(0 <= n_ && n_ <= 100000000);
        base_ = 1;
        log_ = 0;
        while (base_ < n_) { base_ <<= 1; ++log_; }
        data_ = std::vector<S>(2 * base_, e());
        for (int i = 0; i < n_; i++) data_[base_ + i] = v[i];
        for (int i = base_ - 1; i >= 1; i--) {
            update(i);
        }
    }

    void set(int p, S x) {
        assert(0 <= p && p < n_);
        p += base_;
        data_[p] = x;
        for (int i = 1; i <= log_; i++) update(p >> i);
    }

    S get(int p) const {
        assert(0 <= p && p < n_);
        return data_[p + base_];
    }

    S prod(int l, int r) const {
        assert(0 <= l && l <= r && r <= n_);
        S sml = e(), smr = e();
        l += base_;
        r += base_;

        while (l < r) {
            if (l & 1) sml = op(sml, data_[l++]);
            if (r & 1) smr = op(data_[--r], smr);
            l >>= 1;
            r >>= 1;
        }
        return op(sml, smr);
    }

    S all_prod() const { return data_[1]; }

    template <bool (*f)(S)> int max_right(int l) const {
        return max_right(l, [](S x) { return f(x); });
    }
    template <class F> int max_right(int l, F f) const {
        assert(0 <= l && l <= n_);
        assert(f(e()));
        if (l == n_) return n_;
        l += base_;
        S sm = e();
        do {
            while (l % 2 == 0) l >>= 1;
            if (!f(op(sm, data_[l]))) {
                while (l < base_) {
                    l = (2 * l);
                    if (f(op(sm, data_[l]))) {
                        sm = op(sm, data_[l]);
                        l++;
                    }
                }
                return l - base_;
            }
            sm = op(sm, data_[l]);
            l++;
        } while ((l & -l) != l);
        return n_;
    }

    template <bool (*f)(S)> int min_left(int r) const {
        return min_left(r, [](S x) { return f(x); });
    }
    template <class F> int min_left(int r, F f) const {
        assert(0 <= r && r <= n_);
        assert(f(e()));
        if (r == 0) return 0;
        r += base_;
        S sm = e();
        do {
            r--;
            while (r > 1 && (r % 2)) r >>= 1;
            if (!f(op(data_[r], sm))) {
                while (r < base_) {
                    r = (2 * r + 1);
                    if (f(op(data_[r], sm))) {
                        sm = op(data_[r], sm);
                        r--;
                    }
                }
                return r + 1 - base_;
            }
            sm = op(data_[r], sm);
        } while ((r & -r) != r);
        return 0;
    }

    int size() const { return n_; }

  private:
    static std::vector<S> initial_values(int n) {
        assert(0 <= n && n <= 100000000);
        return std::vector<S>(n, e());
    }
    int n_, base_, log_;
    std::vector<S> data_;

    void update(int k) { data_[k] = op(data_[2 * k], data_[2 * k + 1]); }
};

}  // namespace cp
