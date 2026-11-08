#pragma once
// 説明: docs/graph_connectivity/tree_decomposition_width_2.md
#include <algorithm>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <vector>
#include "graph.hpp"

namespace cp {
struct WidthTwoTreeDecomposition {
    std::vector<std::vector<int>> bags; // bag ID = 消去する元頂点ID
    std::vector<int> elimination_order;
    Graph<int> forest;
};
template<class T> std::optional<WidthTwoTreeDecomposition> tree_decomposition_width_2(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("tree decomposition requires undirected graph");
    int n=g.size(); std::vector<std::set<int>> neighbors(n);
    for(const auto& e:g.edges()) {
        if(e.from==e.to || !neighbors[e.from].insert(e.to).second) throw std::invalid_argument("simple graph required");
        neighbors[e.to].insert(e.from);
    }
    std::queue<int> queue; for(int v=0;v<n;++v) if(neighbors[v].size()<=2) queue.push(v);
    WidthTwoTreeDecomposition result; result.bags.resize(n); result.forest=Graph<int>(n);
    std::vector<int> position(n,-1);
    while(!queue.empty()) {
        int v=queue.front(); queue.pop(); if(position[v]>=0 || neighbors[v].size()>2) continue;
        position[v]=int(result.elimination_order.size()); result.elimination_order.push_back(v);
        auto& bag=result.bags[v]; bag.push_back(v); for(int u:neighbors[v]) bag.push_back(u);
        if(bag.size()==3) { neighbors[bag[1]].insert(bag[2]); neighbors[bag[2]].insert(bag[1]); }
        for(int u:neighbors[v]) { neighbors[u].erase(v); if(neighbors[u].size()<=2) queue.push(u); }
        neighbors[v].clear();
    }
    if(int(result.elimination_order.size())!=n) return std::nullopt;
    for(int v=0;v<n;++v) {
        int p=-1; for(int u:result.bags[v]) if(u!=v && (p<0 || position[u]<position[p])) p=u;
        if(p>=0) result.forest.add_edge(v,p);
    }
    return result;
}
} // namespace cp
