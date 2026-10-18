#pragma once
// 説明: docs/graph_connectivity/incremental_scc.md
#include <algorithm>
#include <cassert>
#include <queue>
#include <set>
#include <utility>
#include <vector>
#include "scc.hpp"

namespace cp {
// 縮約DAGを維持。新辺a->bに対する帰路b->aがあるとき交差到達集合を併合。
class IncrementalSCC {
    int n_,count_; std::vector<int> component_; std::vector<std::set<int>> forward_,reverse_;
    std::vector<std::pair<int,int>> edges_;
    std::vector<bool> reachable(int start,const std::vector<std::set<int>>& adjacency) const {
        std::vector<bool> seen(n_); std::vector<int> stack{start}; seen[start]=true;
        while(!stack.empty()) { int v=stack.back(); stack.pop_back(); for(int u:adjacency[v]) if(!seen[u]) { seen[u]=true; stack.push_back(u); } }
        return seen;
    }
    void check(int v) const { assert(0<=v && v<n_); (void)v; }
public:
    explicit IncrementalSCC(int n) : n_(n),count_(n),component_(n),forward_(n),reverse_(n) { for(int v=0;v<n;++v) component_[v]=v; }
    int component_count() const { return count_; }
    bool same(int a,int b) const { check(a); check(b); return component_[a]==component_[b]; }
    bool add_edge(int from,int to) {
        check(from); check(to); edges_.emplace_back(from,to); int a=component_[from],b=component_[to]; if(a==b) return false;
        auto reach=reachable(b,forward_);
        if(!reach[a]) { forward_[a].insert(b); reverse_[b].insert(a); return false; }
        auto back=reachable(a,reverse_); std::vector<bool> merged(n_);
        for(int v=0;v<n_;++v) if(reach[v] && back[v]) { merged[v]=true; if(v!=a) --count_; }
        for(int& c:component_) if(merged[c]) c=a;
        for(auto& row:forward_) row.clear();
        for(auto& row:reverse_) row.clear();
        for(auto [u,v]:edges_) { u=component_[u]; v=component_[v]; if(u!=v) { forward_[u].insert(v); reverse_[v].insert(u); } }
        return true;
    }
    StronglyConnectedComponentsResult components() const {
        std::vector<int> indegree(n_),number(n_,-1); std::queue<int> queue;
        for(int v=0;v<n_;++v) if(component_[v]==v) { indegree[v]=int(reverse_[v].size()); if(!indegree[v]) queue.push(v); }
        int id=0; while(!queue.empty()) { int v=queue.front(); queue.pop(); number[v]=id++; for(int u:forward_[v]) if(--indegree[u]==0) queue.push(u); }
        StronglyConnectedComponentsResult result; result.component.resize(n_); result.groups.resize(count_);
        for(int v=0;v<n_;++v) { int c=number[component_[v]]; result.component[v]=c; result.groups[c].push_back(v); }
        return result;
    }
};
} // namespace cp
