#pragma once
// 説明: docs/range_query/sortable_segtree.md
#include <algorithm>
#include <cassert>
#include <iterator>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include "segtree.hpp"

namespace cp {
// 一意なkey [0,universe) とpayload。key順で区間sortし、payloadを列順に集約。
// sorted runを値域木で分割/併合し、runの積を外側のSegtreeへ置く。
template<class S,auto op,auto e> class SortableSegtree {
    struct Node { int left=0,right=0,count=0; S forward=e(),backward=e(); };
    struct Run { int root; bool descending; };
    int n_,universe_; std::vector<Node> nodes_{Node{}}; std::map<int,Run> runs_; std::set<int> keys_;
    Segtree<S,op,e> outer_;
    int make(Node n) { nodes_.push_back(n); return int(nodes_.size())-1; }
    void pull(int k) {
        int a=nodes_[k].left,b=nodes_[k].right;
        nodes_[k].count=nodes_[a].count+nodes_[b].count;
        nodes_[k].forward=op(nodes_[a].forward,nodes_[b].forward);
        nodes_[k].backward=op(nodes_[b].backward,nodes_[a].backward);
    }
    int singleton(int l,int r,int key,const S& value) {
        if(r-l==1) return make({0,0,1,value,value});
        int m=l+(r-l)/2,k=make(Node{});
        int child=key<m?singleton(l,m,key,value):singleton(m,r,key,value);
        if(key<m) nodes_[k].left=child; else nodes_[k].right=child;
        pull(k); return k;
    }
    std::pair<int,int> split(int k,int count) {
        if(!count) return {0,k};
        if(count==nodes_[k].count) return {k,0};
        int other=make(Node{}),a=nodes_[k].left,b=nodes_[k].right,lc=nodes_[a].count;
        if(count<lc) {
            auto parts=split(a,count); nodes_[k].left=parts.first; nodes_[k].right=0;
            nodes_[other].left=parts.second; nodes_[other].right=b;
        } else if(count==lc) {
            nodes_[k].right=0; nodes_[other].right=b;
        } else {
            auto parts=split(b,count-lc); nodes_[k].right=parts.first;
            nodes_[other].right=parts.second;
        }
        pull(k); pull(other); return {k,other};
    }
    int meld(int a,int b) {
        if(!a || !b) return a?a:b;
        nodes_[a].left=meld(nodes_[a].left,nodes_[b].left);
        nodes_[a].right=meld(nodes_[a].right,nodes_[b].right);
        pull(a); return a;
    }
    S product(Run run) const { return run.descending?nodes_[run.root].backward:nodes_[run.root].forward; }
    typename std::map<int,Run>::iterator cut(int p) {
        if(p==n_) return runs_.end();
        auto it=std::prev(runs_.upper_bound(p));
        if(it->first==p) return it;
        Run run=it->second; int count=p-it->first;
        auto [a,b]=split(run.root,run.descending?nodes_[run.root].count-count:count);
        if(run.descending) std::swap(a,b);
        it->second.root=a; outer_.set(it->first,product(it->second));
        auto next=runs_.emplace(p,Run{b,run.descending}).first; outer_.set(p,product(next->second)); return next;
    }
public:
    SortableSegtree(const std::vector<std::pair<int,S>>& a,int universe) : n_(int(a.size())),universe_(universe),outer_(n_) {
        if(universe<=0) throw std::invalid_argument("positive key universe required");
        std::vector<int> keys;
        for(const auto& item:a) { if(item.first<0 || item.first>=universe) throw std::invalid_argument("key out of range"); keys.push_back(item.first); }
        std::sort(keys.begin(),keys.end());
        if(std::adjacent_find(keys.begin(),keys.end())!=keys.end()) throw std::invalid_argument("unique keys required");
        keys_.insert(keys.begin(),keys.end());
        for(int i=0;i<n_;++i) { runs_.emplace(i,Run{singleton(0,universe_,a[i].first,a[i].second),false}); outer_.set(i,a[i].second); }
    }
    int size() const { return n_; }
    void sort(int l,int r,bool descending=false) {
        assert(0<=l && l<=r && r<=n_); if(l==r) return;
        auto end=cut(r),begin=cut(l); int root=0;
        for(auto it=begin;it!=end;) { root=meld(root,it->second.root); outer_.set(it->first,e()); it=runs_.erase(it); }
        auto it=runs_.emplace(l,Run{root,descending}).first; outer_.set(l,product(it->second));
    }
    S prod(int l,int r) {
        assert(0<=l && l<=r && r<=n_); if(l==r) return e();
        cut(r); cut(l); return outer_.prod(l,r);
    }
    std::pair<int,S> get(int p) {
        assert(0<=p && p<n_); cut(p+1); auto it=cut(p); int k=it->second.root,l=0,r=universe_;
        while(r-l>1) { int m=l+(r-l)/2; if(nodes_[nodes_[k].left].count) { k=nodes_[k].left; r=m; } else { k=nodes_[k].right; l=m; } }
        return {l,nodes_[k].forward};
    }
    void set(int p,int key,const S& value) {
        assert(0<=p && p<n_);
        if(key<0 || key>=universe_) throw std::invalid_argument("key out of range");
        int old=get(p).first;
        if(key!=old && keys_.count(key)) throw std::invalid_argument("unique keys required");
        keys_.erase(old); keys_.insert(key);
        runs_.find(p)->second=Run{singleton(0,universe_,key,value),false};
        outer_.set(p,value);
    }
};
} // namespace cp
