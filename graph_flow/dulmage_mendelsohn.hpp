#pragma once
// 説明: docs/graph_flow/dulmage_mendelsohn.md
#include <algorithm>
#include <queue>
#include <utility>
#include <vector>
#include "bipartite_matching.hpp"
#include "../graph_connectivity/scc.hpp"

namespace cp {
struct DulmageMendelsohnResult {
    BipartiteMatchingResult matching;
    std::vector<int> component; // 左[0,L)、右[L,L+R)。0=右余剰、最後=左余剰。
    std::vector<std::vector<int>> blocks;
    std::vector<bool> allowed_edge; // ある最大マッチングに使える元辺。
};
inline DulmageMendelsohnResult dulmage_mendelsohn(int left_size,int right_size,const std::vector<std::pair<int,int>>& edges) {
    BipartiteMatching problem(left_size,right_size); for(auto [u,v]:edges) problem.add_edge(u,v);
    DulmageMendelsohnResult result; result.matching=problem.maximum_matching(); int n=left_size+right_size;
    Graph<int> alternating(n,GraphKind::Directed); std::vector<std::vector<int>> reverse(n);
    auto add=[&](int u,int v) { alternating.add_edge(u,v); reverse[v].push_back(u); };
    for(auto [u,v]:edges) add(u,left_size+v);
    for(auto [u,v]:result.matching.pairs) add(left_size+v,u);
    auto reach=[&](bool reverse_direction) {
        std::vector<bool> seen(n); std::queue<int> queue;
        if(reverse_direction) { for(int v=0;v<right_size;++v) if(result.matching.right_match[v]<0) { seen[left_size+v]=true; queue.push(left_size+v); } }
        else { for(int u=0;u<left_size;++u) if(result.matching.left_match[u]<0) { seen[u]=true; queue.push(u); } }
        while(!queue.empty()) {
            int v=queue.front(); queue.pop();
            if(reverse_direction) { for(int u:reverse[v]) if(!seen[u]) { seen[u]=true; queue.push(u); } }
            else for(const auto& e:alternating[v]) if(!seen[e.to]) { seen[e.to]=true; queue.push(e.to); }
        }
        return seen;
    };
    auto left_free=reach(false),right_free=reach(true); auto components=scc(alternating);
    std::vector<int> number(components.size(),-1); int next=1;
    for(int c=0;c<components.size();++c) for(int v:components.groups[c]) if(!left_free[v] && !right_free[v]) { number[c]=next++; break; }
    result.blocks.resize(next+1); result.component.resize(n);
    for(int v=0;v<n;++v) {
        int c=right_free[v]?0:left_free[v]?next:number[components.component[v]];
        result.component[v]=c; result.blocks[c].push_back(v);
    }
    for(auto [u,v]:edges) result.allowed_edge.push_back(result.component[u]==result.component[left_size+v]);
    return result;
}
} // namespace cp
