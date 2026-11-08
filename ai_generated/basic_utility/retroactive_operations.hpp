#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../range_query/segtree.hpp"

namespace cp {
// Editable operation timeline over a fixed set of timestamps. M::op(a,b)
// means a followed by b; associativity and M::e() are required, inverses are not.
template<class M> class RetroactiveOperations {
public:
    using S = typename M::S;
    using Time = std::int64_t;
    explicit RetroactiveOperations(std::vector<Time> times)
        : times_(normalize(std::move(times))), tree_(static_cast<int>(times_.size())),
          present_(times_.size(), false) {}

    std::size_t capacity() const { return times_.size(); }
    std::size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }
    bool contains(Time time) const {
        const int p = lower(time);
        return p < static_cast<int>(times_.size()) && times_[p] == time && present_[p];
    }
    std::optional<S> get(Time time) const {
        const int p = position(time);
        if (!present_[p]) return std::nullopt;
        return tree_.get(p);
    }
    // Insert or replace the operation at a registered time.
    void set(Time time, const S& operation) {
        const int p = position(time);
        tree_.set(p, operation);
        if (!present_[p]) { present_[p] = true; ++count_; }
    }
    bool erase(Time time) {
        const int p = position(time);
        if (!present_[p]) return false;
        tree_.set(p, M::e()); present_[p] = false; --count_;
        return true;
    }
    S before(Time time) const { return tree_.prod(0, lower(time)); }
    S through(Time time) const {
        const int p = static_cast<int>(std::upper_bound(times_.begin(), times_.end(), time) - times_.begin());
        return tree_.prod(0, p);
    }
    S fold(Time first, Time last) const {
        if (last < first) throw std::invalid_argument("reversed operation time range");
        return tree_.prod(lower(first), lower(last));
    }
    S all() const { return tree_.all_prod(); }

private:
    std::vector<Time> times_;
    Segtree<S, M::op, M::e> tree_;
    std::vector<bool> present_;
    std::size_t count_ = 0;
    static std::vector<Time> normalize(std::vector<Time> times) {
        std::sort(times.begin(), times.end());
        times.erase(std::unique(times.begin(), times.end()), times.end());
        if (times.size() > 100000000) throw std::length_error("too many operation timestamps");
        return times;
    }
    int lower(Time time) const {
        return static_cast<int>(std::lower_bound(times_.begin(), times_.end(), time) - times_.begin());
    }
    int position(Time time) const {
        const int p = lower(time);
        if (p == static_cast<int>(times_.size()) || times_[p] != time)
            throw std::out_of_range("unregistered operation timestamp");
        return p;
    }
};
} // namespace cp
