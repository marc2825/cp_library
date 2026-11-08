#pragma once
#include <algorithm>
#include <cassert>
#include <optional>
#include <vector>

namespace cp {
// 各prefixの頻度木を永続化。静的区間のk番目・個数、構築O(n log n)。
template<class T> class PersistentOrderStatistics {
    struct Node { int left=0,right=0,count=0; };
    std::vector<Node> nodes_{Node{}}; std::vector<int> roots_{0}; std::vector<T> values_;
    int insert(int root,int l,int r,int p) {
        int k=int(nodes_.size()); nodes_.push_back(nodes_[root]); ++nodes_[k].count;
        if(r-l==1) return k;
        int m=l+(r-l)/2;
        if(p<m) { int child=insert(nodes_[k].left,l,m,p); nodes_[k].left=child; }
        else { int child=insert(nodes_[k].right,m,r,p); nodes_[k].right=child; }
        return k;
    }
    int prefix(int a,int b,int l,int r,int end) const {
        if(end<=l) return 0;
        if(r<=end) return nodes_[b].count-nodes_[a].count;
        int m=l+(r-l)/2;
        return prefix(nodes_[a].left,nodes_[b].left,l,m,end)+prefix(nodes_[a].right,nodes_[b].right,m,r,end);
    }
    void bounds(int l,int r) const { assert(0<=l && l<=r && r<=size()); (void)l; (void)r; }
public:
    explicit PersistentOrderStatistics(const std::vector<T>& a={}) : values_(a) {
        std::sort(values_.begin(),values_.end());
        values_.erase(std::unique(values_.begin(),values_.end(),[](const T& x,const T& y){return !(x<y)&&!(y<x);}),values_.end());
        for(const auto &x:a) roots_.push_back(insert(roots_.back(),0,int(values_.size()),int(std::lower_bound(values_.begin(),values_.end(),x)-values_.begin())));
    }
    int size() const { return int(roots_.size())-1; }
    std::optional<T> kth_smallest(int l,int r,int k) const {
        bounds(l,r); if(k<0 || k>=r-l) return std::nullopt;
        int a=roots_[l],b=roots_[r],lo=0,hi=int(values_.size());
        while(hi-lo>1) {
            int count=nodes_[nodes_[b].left].count-nodes_[nodes_[a].left].count,m=lo+(hi-lo)/2;
            if(k<count) { a=nodes_[a].left; b=nodes_[b].left; hi=m; }
            else { k-=count; a=nodes_[a].right; b=nodes_[b].right; lo=m; }
        }
        return values_[lo];
    }
    int count_less(int l,int r,const T& x) const {
        bounds(l,r); if(values_.empty()) return 0;
        return prefix(roots_[l],roots_[r],0,int(values_.size()),int(std::lower_bound(values_.begin(),values_.end(),x)-values_.begin()));
    }
    int range_freq(int l,int r,const T& low,const T& high) const { return high<low?0:count_less(l,r,high)-count_less(l,r,low); }
};
} // namespace cp
