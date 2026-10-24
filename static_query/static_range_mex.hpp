#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace cp {
// prefix ごとの最終出現位置を永続化。mex(l,r) は O(log(n+1))。
class StaticRangeMex {
    struct Node { int left = 0, right = 0, last = -1; };
    int n_;
    std::vector<Node> nodes_{Node{}};
    std::vector<int> roots_{0};
    int update(int root, int l, int r, int p, int time) {
        if (nodes_.size() >= std::size_t(std::numeric_limits<int>::max()))
            throw std::length_error("StaticRangeMex node limit");
        int k = int(nodes_.size());
        nodes_.push_back(nodes_[root]);
        if (r-l == 1) { nodes_[k].last = time; return k; }
        int m = l + (r-l)/2;
        if (p < m) { int c = update(nodes_[k].left,l,m,p,time); nodes_[k].left = c; }
        else { int c = update(nodes_[k].right,m,r,p,time); nodes_[k].right = c; }
        nodes_[k].last = std::min(nodes_[nodes_[k].left].last,nodes_[nodes_[k].right].last);
        return k;
    }
public:
    template<class Integer> explicit StaticRangeMex(const std::vector<Integer>& values) : n_(0) {
        static_assert(std::is_integral_v<Integer>, "integral values required");
        if (values.size() >= std::size_t(std::numeric_limits<int>::max()))
            throw std::length_error("StaticRangeMex size limit");
        n_ = int(values.size()); roots_.reserve(values.size()+1);
        for (int i=0; i<n_; ++i) {
            auto x = values[i]; int root = roots_.back();
            bool nonnegative = true;
            if constexpr (std::is_signed_v<Integer>) nonnegative = x >= 0;
            if (nonnegative && static_cast<unsigned long long>(x) <= static_cast<unsigned long long>(n_))
                root = update(root,0,n_+1,int(x),i);
            roots_.push_back(root);
        }
    }
    int size() const { return n_; }
    std::size_t node_count() const { return nodes_.size()-1; }
    int mex(int l, int r) const {
        assert(0<=l && l<=r && r<=n_);
        int k = roots_[r], a = 0, b = n_+1;
        while (b-a > 1) {
            int m = a+(b-a)/2;
            if (nodes_[nodes_[k].left].last < l) { k = nodes_[k].left; b = m; }
            else { k = nodes_[k].right; a = m; }
        }
        return a;
    }
};
} // namespace cp
