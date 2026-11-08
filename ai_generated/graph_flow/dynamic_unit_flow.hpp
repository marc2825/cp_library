#pragma once
// 説明: docs/graph_flow/dynamic_unit_flow.md
#include <algorithm>
#include <queue>
#include <stdexcept>
#include <vector>
namespace cp {
// 有向単位容量の追加/有効化/無効化。更新ごと O(n+m)、辺IDは不変。
class DynamicUnitFlow {
    struct Arc { int from,to,cap; };
    int n_,s_,t_,value_=0; std::vector<Arc> arcs_; std::vector<std::vector<int>> adj_; std::vector<bool> enabled_;
    void vertex(int v) const { if(v<0 || v>=n_) throw std::out_of_range("flow vertex"); }
    void push(int id) { --arcs_[id].cap; ++arcs_[id^1].cap; }
    bool augment(int source,int sink,bool repair=false) {
        if(source==sink) return true;
        std::vector<int> parent(n_,-1); std::queue<int> q; q.push(source); parent[source]=-2;
        while(!q.empty() && parent[sink]==-1) {
            int v=q.front(); q.pop();
            for(int id:adj_[v]) if((repair || id>=2) && arcs_[id].cap && parent[arcs_[id].to]==-1) {
                parent[arcs_[id].to]=id; q.push(arcs_[id].to);
            }
        }
        if(parent[sink]==-1) return false;
        for(int v=sink;v!=source;v=arcs_[parent[v]].from) push(parent[v]);
        return true;
    }
public:
    struct Edge { int from,to,capacity,flow; };
    DynamicUnitFlow(int n,int source,int sink):n_(n),s_(source),t_(sink) {
        if(n<0) throw std::invalid_argument("negative flow size");
        vertex(s_); vertex(t_);
        if(s_==t_) throw std::invalid_argument("distinct terminals required");
        adj_.resize(n); arcs_={{t_,s_,0},{s_,t_,0}}; adj_[t_].push_back(0); adj_[s_].push_back(1);
    }
    int value() const { return value_; }
    int edge_count() const { return int(enabled_.size()); }
    int add_edge(int u,int v,bool enabled=true) {
        vertex(u); vertex(v); int id=int(arcs_.size()); arcs_.push_back({u,v,enabled?1:0}); arcs_.push_back({v,u,0});
        adj_[u].push_back(id); adj_[v].push_back(id+1); enabled_.push_back(enabled);
        if(enabled && augment(s_,t_)) ++value_;
        return edge_count()-1;
    }
    Edge get_edge(int id) const {
        if(id<0 || id>=edge_count()) throw std::out_of_range("flow edge");
        auto e=arcs_[2+2*id]; return {e.from,e.to,enabled_[id]?1:0,arcs_[3+2*id].cap};
    }
    int set_enabled(int id,bool enabled) {
        auto e=get_edge(id); if(enabled_[id]==enabled) return value_; enabled_[id]=enabled;
        int a=2+2*id;
        if(enabled) { arcs_[a].cap=1; if(augment(s_,t_)) ++value_; }
        else {
            arcs_[a].cap=0;
            if(e.flow) {
                arcs_[a^1].cap=0; arcs_[1].cap=value_;
                if(!augment(e.from,e.to,true)) throw std::logic_error("unit flow repair failed");
                value_=arcs_[1].cap; arcs_[0].cap=arcs_[1].cap=0;
                if(augment(s_,t_)) ++value_;
            }
        }
        return value_;
    }
    std::vector<bool> min_cut() const {
        std::vector<bool> seen(n_); std::queue<int> q; seen[s_]=true; q.push(s_);
        while(!q.empty()) { int v=q.front(); q.pop(); for(int id:adj_[v]) if(id>=2 && arcs_[id].cap && !seen[arcs_[id].to]) { seen[arcs_[id].to]=true; q.push(arcs_[id].to); } }
        return seen;
    }
};
} // namespace cp
