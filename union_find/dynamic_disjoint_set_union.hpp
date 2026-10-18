#pragma once
// 説明: docs/union_find/dynamic_disjoint_set_union.md
#include <functional>
#include <map>
#include <numeric>
#include <utility>
#include <vector>

namespace cp {
// 任意ラベルを必要時に追加。辺削除はOfflineDynamicConnectivityへ。
template<class Key,class Compare=std::less<Key>> class DynamicDisjointSetUnion {
    std::map<Key,int,Compare> index_; std::vector<int> parent_; int components_=0;
    int root(int v) { return parent_[v]<0?v:parent_[v]=root(parent_[v]); }
public:
    explicit DynamicDisjointSetUnion(Compare comp=Compare{}) : index_(comp) {}
    int add(const Key& key) {
        auto it=index_.find(key); if (it!=index_.end()) return it->second;
        int v=int(parent_.size()); index_.emplace(key,v); parent_.push_back(-1); ++components_; return v;
    }
    bool contains(const Key& key) const { return index_.find(key)!=index_.end(); }
    int vertex_count() const { return int(parent_.size()); }
    int component_count() const { return components_; }
    bool merge(const Key& a,const Key& b) {
        int u=root(add(a)),v=root(add(b)); if(u==v) return false;
        if(parent_[u]>parent_[v]) std::swap(u,v);
        parent_[u]+=parent_[v]; parent_[v]=u; --components_; return true;
    }
    bool same(const Key& a,const Key& b) { int u=add(a),v=add(b); return root(u)==root(v); }
    int size(const Key& a) { return -parent_[root(add(a))]; }
};
} // namespace cp
