#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>
#include "graph.hpp"

namespace cp {
struct EulerianTrailResult { std::vector<int> vertices,edge_ids; };
// 有向/無向、多重辺・自己ループ・孤立点可。start=-1で自動選択。
template<class T> std::optional<EulerianTrailResult> eulerian_trail(const Graph<T>& g,int start=-1) {
    int n=g.size(),m=g.edge_count(); assert(start==-1 || (0<=start && start<n));
    if(!n) return EulerianTrailResult{};
    std::vector<int> degree(n),out(n),in(n),odd; int first=-1,required=-1,ends=0;
    for(const auto& e:g.edges()) { ++out[e.from]; ++in[e.to]; ++degree[e.from]; ++degree[e.to]; }
    for(int v=0;v<n;++v) {
        if((g.directed()?out[v]:degree[v]) && first==-1) first=v;
        if(g.directed()) {
            int delta=out[v]-in[v];
            if(delta==1) { required=v; ++ends; } else if(delta==-1) ++ends;
            else if(delta!=0) return std::nullopt;
        } else if(degree[v]&1) odd.push_back(v);
    }
    if(g.directed()) { if(ends!=0 && (ends!=2 || required==-1)) return std::nullopt; }
    else { if(odd.size()!=0 && odd.size()!=2) return std::nullopt; if(!odd.empty()) required=odd[0]; }
    if(start==-1) start=required!=-1?required:(first!=-1?first:0);
    if(g.directed() && required!=-1 && start!=required) return std::nullopt;
    if(!g.directed() && !odd.empty() && start!=odd[0] && start!=odd[1]) return std::nullopt;
    std::vector<bool> used(m,false); std::vector<std::size_t> cursor(n);
    std::vector<std::pair<int,int>> stack{{start,-1}}; EulerianTrailResult result;
    while(!stack.empty()) {
        int v=stack.back().first;
        while(cursor[v]<g[v].size() && used[g[v][cursor[v]].idx]) ++cursor[v];
        if(cursor[v]<g[v].size()) { auto e=g[v][cursor[v]++]; used[e.idx]=true; stack.emplace_back(e.to,e.idx); }
        else { result.vertices.push_back(v); if(stack.back().second!=-1) result.edge_ids.push_back(stack.back().second); stack.pop_back(); }
    }
    if(int(result.edge_ids.size())!=m) return std::nullopt;
    std::reverse(result.vertices.begin(),result.vertices.end()); std::reverse(result.edge_ids.begin(),result.edge_ids.end());
    return result;
}
} // namespace cp
