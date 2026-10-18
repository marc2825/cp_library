#pragma once
// 説明: docs/graph_connectivity/dominator_tree.md
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>
#include "graph.hpp"

namespace cp {
struct DominatorTreeResult {
    std::vector<int> idom,tin,tout;
    bool dominates(int u,int v) const {
        assert(0<=u && u<int(idom.size()) && 0<=v && v<int(idom.size()));
        return tin[u]>=0 && tin[v]>=tin[u] && tin[v]<tout[u];
    }
};
// Lengauer–Tarjanのlink/eval。DFSと経路圧縮は非再帰。O((V+E)log V)。
template<class T> DominatorTreeResult dominator_tree(const Graph<T>& g,int root=0) {
    if(!g.directed()) throw std::invalid_argument("dominator tree requires directed graph");
    int n=g.size(); DominatorTreeResult result; result.idom.assign(n,-1); result.tin.assign(n,-1); result.tout.assign(n,-1);
    if(!n) return result;
    assert(0<=root && root<n);
    std::vector<int> number(n,-1),vertex{root},parent{-1}; number[root]=0;
    std::vector<std::pair<int,std::size_t>> stack{{root,0}};
    while(!stack.empty()) {
        int v=stack.back().first; auto& next=stack.back().second;
        if(next==g[v].size()) { stack.pop_back(); continue; }
        int u=g[v][next++].to; if(number[u]>=0) continue;
        number[u]=int(vertex.size()); vertex.push_back(u); parent.push_back(number[v]); stack.emplace_back(u,0);
    }
    int count=int(vertex.size()); std::vector<std::vector<int>> predecessors(count),bucket(count);
    for(const auto& e:g.edges()) if(number[e.from]>=0 && number[e.to]>=0) predecessors[number[e.to]].push_back(number[e.from]);
    std::vector<int> semi(count),label(count),ancestor(count,-1),idom(count,-1);
    std::iota(semi.begin(),semi.end(),0); label=semi;
    auto eval=[&](int v) {
        std::vector<int> path;
        for(int x=v;ancestor[x]>=0 && ancestor[ancestor[x]]>=0;x=ancestor[x]) path.push_back(x);
        for(auto it=path.rbegin();it!=path.rend();++it) {
            int x=*it,p=ancestor[x]; if(semi[label[p]]<semi[label[x]]) label[x]=label[p]; ancestor[x]=ancestor[p];
        }
        return label[v];
    };
    for(int w=count-1;w>0;--w) {
        for(int v:predecessors[w]) semi[w]=std::min(semi[w],semi[eval(v)]);
        bucket[semi[w]].push_back(w); ancestor[w]=parent[w];
        for(int v:bucket[parent[w]]) { int u=eval(v); idom[v]=semi[u]<semi[v]?u:parent[w]; }
        bucket[parent[w]].clear();
    }
    for(int w=1;w<count;++w) if(idom[w]!=semi[w]) idom[w]=idom[idom[w]];
    result.idom[root]=root; std::vector<std::vector<int>> children(n);
    for(int w=1;w<count;++w) { int v=vertex[w],p=vertex[idom[w]]; result.idom[v]=p; children[p].push_back(v); }
    int timer=0; stack={{root,0}}; result.tin[root]=timer++;
    while(!stack.empty()) {
        int v=stack.back().first; auto& next=stack.back().second;
        if(next==children[v].size()) { result.tout[v]=timer; stack.pop_back(); }
        else { int u=children[v][next++]; result.tin[u]=timer++; stack.emplace_back(u,0); }
    }
    return result;
}
} // namespace cp
