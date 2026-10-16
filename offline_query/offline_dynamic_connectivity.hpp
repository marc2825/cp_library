#pragma once
#include <algorithm>
#include <cassert>
#include <map>
#include <utility>
#include <vector>
#include "../union_find/rollback_disjoint_set_union.hpp"

namespace cp {
// 無向多重グラフ。add/remove/query を時系列に登録、solve は質問ID順の連結判定。
// 存在しない辺のremoveはfalseで時刻も進めない。solveは繰返し可。
class OfflineDynamicConnectivity {
    struct Event { int type,u,v; };
    int n_; std::vector<Event> events_; std::map<std::pair<int,int>,int> active_; int queries_=0;
    std::pair<int,int> edge(int u,int v) const { assert(0<=u && u<n_ && 0<=v && v<n_); return std::minmax(u,v); }
public:
    explicit OfflineDynamicConnectivity(int n):n_(n) { assert(n>=0); }
    void add_edge(int u,int v) { auto e=edge(u,v); ++active_[e]; events_.push_back({0,e.first,e.second}); }
    bool remove_edge(int u,int v) {
        auto e=edge(u,v); auto it=active_.find(e); if(it==active_.end()) return false;
        if(--it->second==0) active_.erase(it);
        events_.push_back({1,e.first,e.second}); return true;
    }
    int add_query(int u,int v) { edge(u,v); events_.push_back({2,u,v}); return queries_++; }
    int query_count() const { return queries_; }
    std::vector<bool> solve() const {
        int q=int(events_.size()),base=1; while(base<q) base*=2;
        std::vector<std::vector<std::pair<int,int>>> seg(2*base);
        auto insert=[&](int l,int r,std::pair<int,int> e) { for(l+=base,r+=base;l<r;l/=2,r/=2) { if(l&1) seg[l++].push_back(e); if(r&1) seg[--r].push_back(e); } };
        std::map<std::pair<int,int>,std::pair<int,int>> open;
        std::vector<int> id(q,-1); int next=0;
        for(int t=0;t<q;++t) {
            auto e=std::make_pair(events_[t].u,events_[t].v);
            if(events_[t].type==0) { auto &s=open[e]; if(s.first++==0) s.second=t; }
            else if(events_[t].type==1) { auto &s=open[e]; if(--s.first==0) { insert(s.second,t,e); open.erase(e); } }
            else id[t]=next++;
        }
        for(auto [e,s]:open) insert(s.second,q,e);
        RollbackDisjointSetUnion dsu(n_); std::vector<bool> answer(queries_);
        auto dfs=[&](auto&& self,int k)->void {
            auto state=dsu.snapshot(); for(auto [u,v]:seg[k]) dsu.merge(u,v);
            if(k>=base) { int t=k-base; if(t<q && id[t]!=-1) answer[id[t]]=dsu.same(events_[t].u,events_[t].v); }
            else { self(self,2*k); self(self,2*k+1); }
            dsu.rollback(state);
        };
        dfs(dfs,1); return answer;
    }
};
} // namespace cp
