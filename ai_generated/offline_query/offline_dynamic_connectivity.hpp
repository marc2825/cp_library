#pragma once
#include <algorithm>
#include <cassert>
#include <map>
#include <utility>
#include <vector>
#include "../union_find/rollback_disjoint_set_union.hpp"
#include "segment_tree_over_time.hpp"

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
        int q=int(events_.size());
        SegmentTreeOverTime<std::pair<int,int>> seg(q);
        std::map<std::pair<int,int>,std::pair<int,int>> open;
        std::vector<int> id(q,-1); int next=0;
        for(int t=0;t<q;++t) {
            auto e=std::make_pair(events_[t].u,events_[t].v);
            if(events_[t].type==0) { auto &s=open[e]; if(s.first++==0) s.second=t; }
            else if(events_[t].type==1) { auto &s=open[e]; if(--s.first==0) { seg.add(s.second,t,e); open.erase(e); } }
            else id[t]=next++;
        }
        for(auto [e,s]:open) seg.add(s.second,q,e);
        RollbackDisjointSetUnion dsu(n_); std::vector<bool> answer(queries_);
        seg.run(dsu,[](auto& d,const auto& e){ d.merge(e.first,e.second); },
            [&](auto& d,int t){ if(id[t]!=-1) answer[id[t]]=d.same(events_[t].u,events_[t].v); });
        return answer;
    }
};
} // namespace cp
