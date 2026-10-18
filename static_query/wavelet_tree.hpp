#pragma once
// 説明: docs/static_query/wavelet_tree.md
#include <algorithm>
#include <cassert>
#include <optional>
#include <vector>

namespace cp {
template<class T> class WaveletTree {
    struct Node { int left=-1,right=-1; std::vector<int> prefix; };
    int n_; std::vector<T> values_; std::vector<Node> nodes_;
    int build(const std::vector<int>& a,int lo,int hi) {
        int k=int(nodes_.size()); nodes_.push_back(Node{});
        if(hi-lo<=1) return k;
        int mid=lo+(hi-lo)/2; std::vector<int> left,right,prefix(1,0);
        for(int x:a) { (x<mid?left:right).push_back(x); prefix.push_back(int(left.size())); }
        nodes_[k].prefix=std::move(prefix);
        int x=build(left,lo,mid),y=build(right,mid,hi); nodes_[k].left=x; nodes_[k].right=y; return k;
    }
    int less(int k,int lo,int hi,int l,int r,int x) const {
        if(!k && values_.empty()) return 0;
        if(x<=lo || l==r) return 0;
        if(hi<=x) return r-l;
        int mid=lo+(hi-lo)/2,a=nodes_[k].prefix[l],b=nodes_[k].prefix[r];
        return less(nodes_[k].left,lo,mid,a,b,x)+less(nodes_[k].right,mid,hi,l-a,r-b,x);
    }
    void bounds(int l,int r) const { assert(0<=l && l<=r && r<=n_); (void)l; (void)r; }
public:
    explicit WaveletTree(const std::vector<T>& a={}) : n_(int(a.size())),values_(a) {
        std::sort(values_.begin(),values_.end()); values_.erase(std::unique(values_.begin(),values_.end(),[](const T& x,const T& y){return !(x<y)&&!(y<x);}),values_.end());
        std::vector<int> ranks; for(const T& x:a) ranks.push_back(int(std::lower_bound(values_.begin(),values_.end(),x)-values_.begin()));
        build(ranks,0,int(values_.size()));
    }
    int size() const { return n_; }
    int count_less(int l,int r,const T& x) const { bounds(l,r); return less(0,0,int(values_.size()),l,r,int(std::lower_bound(values_.begin(),values_.end(),x)-values_.begin())); }
    int range_freq(int l,int r,const T& lower,const T& upper) const { bounds(l,r); return upper<lower?0:count_less(l,r,upper)-count_less(l,r,lower); }
    std::optional<T> kth_smallest(int l,int r,int rank) const {
        bounds(l,r); if(rank<0 || rank>=r-l) return std::nullopt;
        int k=0,lo=0,hi=int(values_.size());
        while(hi-lo>1) {
            int a=nodes_[k].prefix[l],b=nodes_[k].prefix[r],mid=lo+(hi-lo)/2;
            if(rank<b-a) { l=a; r=b; k=nodes_[k].left; hi=mid; }
            else { rank-=b-a; l-=a; r-=b; k=nodes_[k].right; lo=mid; }
        }
        return values_[lo];
    }
};
} // namespace cp
