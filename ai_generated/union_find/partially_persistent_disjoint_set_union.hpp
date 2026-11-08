#pragma once
#include <iterator>
#include <algorithm>
#include <cassert>
#include <limits>
#include <utility>
#include <vector>

namespace cp {
// merge呼出しごとに時刻を進める。更新は最新版だけ、過去の問い合わせは任意。
class PartiallyPersistentDisjointSetUnion {
    std::vector<int> parent_, linked_at_;
    std::vector<std::vector<std::pair<int,int>>> sizes_;
    int time_=0;
public:
    explicit PartiallyPersistentDisjointSetUnion(int n):parent_(n,-1),
        linked_at_(n,std::numeric_limits<int>::max()),sizes_(n,{{0,1}}) { assert(n>=0); }
    int time() const { return time_; }
    int vertex_count() const { return int(parent_.size()); }
    int leader(int t,int v) const {
        assert(0<=t && t<=time_ && 0<=v && v<vertex_count());
        while (parent_[v]!=-1 && linked_at_[v]<=t) v=parent_[v];
        return v;
    }
    bool same(int t,int u,int v) const { return leader(t,u)==leader(t,v); }
    int size(int t,int v) const {
        v=leader(t,v); const auto& a=sizes_[v];
        return std::prev(std::upper_bound(a.begin(),a.end(),std::make_pair(t,std::numeric_limits<int>::max())))->second;
    }
    bool merge(int u,int v) {
        assert(time_<std::numeric_limits<int>::max());
        u=leader(time_,u); v=leader(time_,v); ++time_;
        if (u==v) return false;
        if (sizes_[u].back().second<sizes_[v].back().second) std::swap(u,v);
        parent_[v]=u; linked_at_[v]=time_;
        sizes_[u].emplace_back(time_,sizes_[u].back().second+sizes_[v].back().second);
        return true;
    }
};
} // namespace cp
