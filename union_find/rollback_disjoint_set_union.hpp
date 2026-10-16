#pragma once
#include <cassert>
#include <cstddef>
#include <numeric>
#include <utility>
#include <vector>

namespace cp {
// 経路圧縮なし。merge は成功・失敗とも履歴1個。snapshot は履歴長。
class RollbackDisjointSetUnion {
    std::vector<int> parent_,size_; std::vector<std::pair<int,int>> history_; int components_;
public:
    explicit RollbackDisjointSetUnion(int n=0):parent_(checked(n)),size_(n,1),components_(n) { std::iota(parent_.begin(),parent_.end(),0); }
    int size() const { return int(parent_.size()); }
    int leader(int v) const { assert(0<=v && v<size()); while(parent_[v]!=v) v=parent_[v]; return v; }
    bool same(int u,int v) const { return leader(u)==leader(v); }
    int component_size(int v) const { return size_[leader(v)]; }
    int component_count() const { return components_; }
    std::size_t snapshot() const { return history_.size(); }
    bool merge(int u,int v) {
        u=leader(u); v=leader(v); if(u==v) { history_.emplace_back(-1,-1); return false; }
        if(size_[u]<size_[v]) std::swap(u,v);
        history_.emplace_back(u,v); parent_[v]=u; size_[u]+=size_[v]; --components_; return true;
    }
    void undo() {
        assert(!history_.empty()); auto [u,v]=history_.back(); history_.pop_back();
        if(u!=-1) { size_[u]-=size_[v]; parent_[v]=v; ++components_; }
    }
    void rollback(std::size_t state) { assert(state<=snapshot()); while(snapshot()>state) undo(); }
private:
    static int checked(int n) { assert(n>=0); return n; }
};
} // namespace cp
