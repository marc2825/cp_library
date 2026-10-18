#pragma once
// 説明: docs/range_query/segtree_fractional_cascading.md
#include <algorithm>
#include <cassert>
#include <iterator>
#include <vector>

namespace cp {
// Merge Sort Treeの各prefixを子のprefixへ結ぶ。count O(log n)。
template<class T> class SegtreeFractionalCascading {
    struct Node { std::vector<T> values; std::vector<int> left; };
    int n_,base_=1; std::vector<Node> tree_;
    int query(int k,int l,int r,int a,int b,int lo,int hi) const {
        if(r<=a || b<=l) return 0;
        if(a<=l && r<=b) return hi-lo;
        int m=l+(r-l)/2,x=tree_[k].left[lo],y=tree_[k].left[hi];
        return query(2*k,l,m,a,b,x,y)+query(2*k+1,m,r,a,b,lo-x,hi-y);
    }
public:
    explicit SegtreeFractionalCascading(const std::vector<T>& a={}) : n_(int(a.size())) {
        while(base_<n_) base_*=2;
        tree_.resize(2*base_);
        for(int i=0;i<n_;++i) tree_[base_+i].values.push_back(a[i]);
        for(int k=base_-1;k;--k) {
            auto& n=tree_[k]; const auto& a0=tree_[2*k].values; const auto& a1=tree_[2*k+1].values;
            std::size_t i=0,j=0; n.left.push_back(0);
            while(i<a0.size() || j<a1.size()) {
                if(j==a1.size() || (i<a0.size() && !(a1[j]<a0[i]))) n.values.push_back(a0[i++]);
                else n.values.push_back(a1[j++]);
                n.left.push_back(int(i));
            }
        }
    }
    int size() const { return n_; }
    int range_freq(int l,int r,const T& lower,const T& upper) const {
        assert(0<=l && l<=r && r<=n_); if(!(lower<upper)) return 0;
        const auto& a=tree_[1].values;
        int lo=int(std::lower_bound(a.begin(),a.end(),lower)-a.begin());
        int hi=int(std::lower_bound(a.begin(),a.end(),upper)-a.begin());
        return query(1,0,base_,l,r,lo,hi);
    }
};
} // namespace cp
