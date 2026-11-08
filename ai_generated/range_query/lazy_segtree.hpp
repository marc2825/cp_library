#pragma once
#include <cassert>
#include <vector>

// Range actions / ordered range product of a monoid. 0-based, [l, r).
// composition(f, g) means f after g. Store segment length in S when needed.
// Docs: docs/range_query/lazy_segtree.md
// Adapted from AtCoder Library (CC0): https://github.com/atcoder/ac-library
namespace cp {

template <class S,
          auto op,
          auto e,
          class F,
          auto mapping,
          auto composition,
          auto id>
struct LazySegtree {
  public:
    LazySegtree() : LazySegtree(0) {}
    explicit LazySegtree(int n) : LazySegtree(initial_values(n)) {}
    explicit LazySegtree(const std::vector<S>& v) : n_(int(v.size())) {
        assert(0 <= n_ && n_ <= 100000000);
        base_ = 1;
        log_ = 0;
        while (base_ < n_) { base_ <<= 1; ++log_; }
        data_ = std::vector<S>(2 * base_, e());
        lazy_ = std::vector<F>(base_, id());
        for (int i = 0; i < n_; i++) data_[base_ + i] = v[i];
        for (int i = base_ - 1; i >= 1; i--) {
            update(i);
        }
    }

    void set(int p, S x) {
        assert(0 <= p && p < n_);
        p += base_;
        for (int i = log_; i >= 1; i--) push(p >> i);
        data_[p] = x;
        for (int i = 1; i <= log_; i++) update(p >> i);
    }

    S get(int p) {
        assert(0 <= p && p < n_);
        p += base_;
        for (int i = log_; i >= 1; i--) push(p >> i);
        return data_[p];
    }

    S prod(int l, int r) {
        assert(0 <= l && l <= r && r <= n_);
        if (l == r) return e();

        l += base_;
        r += base_;

        for (int i = log_; i >= 1; i--) {
            if (((l >> i) << i) != l) push(l >> i);
            if (((r >> i) << i) != r) push((r - 1) >> i);
        }

        S sml = e(), smr = e();
        while (l < r) {
            if (l & 1) sml = op(sml, data_[l++]);
            if (r & 1) smr = op(data_[--r], smr);
            l >>= 1;
            r >>= 1;
        }

        return op(sml, smr);
    }

    S all_prod() const { return data_[1]; }

    void apply(int p, F f) {
        assert(0 <= p && p < n_);
        p += base_;
        for (int i = log_; i >= 1; i--) push(p >> i);
        data_[p] = mapping(f, data_[p]);
        for (int i = 1; i <= log_; i++) update(p >> i);
    }
    void apply(int l, int r, F f) {
        assert(0 <= l && l <= r && r <= n_);
        if (l == r) return;

        l += base_;
        r += base_;

        for (int i = log_; i >= 1; i--) {
            if (((l >> i) << i) != l) push(l >> i);
            if (((r >> i) << i) != r) push((r - 1) >> i);
        }

        {
            int l2 = l, r2 = r;
            while (l < r) {
                if (l & 1) all_apply(l++, f);
                if (r & 1) all_apply(--r, f);
                l >>= 1;
                r >>= 1;
            }
            l = l2;
            r = r2;
        }

        for (int i = 1; i <= log_; i++) {
            if (((l >> i) << i) != l) update(l >> i);
            if (((r >> i) << i) != r) update((r - 1) >> i);
        }
    }

    template <bool (*g)(S)> int max_right(int l) {
        return max_right(l, [](S x) { return g(x); });
    }
    template <class G> int max_right(int l, G g) {
        assert(0 <= l && l <= n_);
        assert(g(e()));
        if (l == n_) return n_;
        l += base_;
        for (int i = log_; i >= 1; i--) push(l >> i);
        S sm = e();
        do {
            while (l % 2 == 0) l >>= 1;
            if (!g(op(sm, data_[l]))) {
                while (l < base_) {
                    push(l);
                    l = (2 * l);
                    if (g(op(sm, data_[l]))) {
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

    template <bool (*g)(S)> int min_left(int r) {
        return min_left(r, [](S x) { return g(x); });
    }
    template <class G> int min_left(int r, G g) {
        assert(0 <= r && r <= n_);
        assert(g(e()));
        if (r == 0) return 0;
        r += base_;
        for (int i = log_; i >= 1; i--) push((r - 1) >> i);
        S sm = e();
        do {
            r--;
            while (r > 1 && (r % 2)) r >>= 1;
            if (!g(op(data_[r], sm))) {
                while (r < base_) {
                    push(r);
                    r = (2 * r + 1);
                    if (g(op(data_[r], sm))) {
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
    std::vector<F> lazy_;

    void update(int k) { data_[k] = op(data_[2 * k], data_[2 * k + 1]); }
    void all_apply(int k, F f) {
        data_[k] = mapping(f, data_[k]);
        if (k < base_) lazy_[k] = composition(f, lazy_[k]);
    }
    void push(int k) {
        all_apply(2 * k, lazy_[k]);
        all_apply(2 * k + 1, lazy_[k]);
        lazy_[k] = id();
    }
};

}  // namespace cp
