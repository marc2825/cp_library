#pragma once
#include <algorithm>
#include <list>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <vector>
#include "graph.hpp"

namespace cp {
struct STNumberingResult {
    std::vector<int> order, rank;
    std::vector<std::pair<int,int>> oriented_edges; // Original edge ID order.
};
// Loopless undirected multigraph. A virtual st edge permits nonadjacent terminals.
// Returns nullopt iff no bipolar ordering exists. Iterative DFS, O(V+E).
template<class T> std::optional<STNumberingResult> st_numbering(const Graph<T>& g,int s,int t) {
    int n=g.size();
    if (g.directed()) throw std::invalid_argument("st numbering requires undirected graph");
    if (s<0 || t<0 || s>=n || t>=n || s==t) throw std::invalid_argument("distinct valid st terminals required");
    std::vector<std::vector<int>> adj(n);
    adj[s].push_back(t); adj[t].push_back(s);
    for (const auto& e:g.edges()) {
        if (e.from==e.to) throw std::invalid_argument("bipolar orientation cannot contain loops");
        adj[e.from].push_back(e.to); adj[e.to].push_back(e.from);
    }
    std::vector<int> pre(n,-1),low(n),parent(n,-1),dfs;
    std::vector<std::size_t> next(n);
    pre[s]=low[s]=0; dfs.push_back(s); std::vector<int> stack{s};
    while (!stack.empty()) {
        int v=stack.back();
        if (next[v]<adj[v].size()) {
            int w=adj[v][next[v]++];
            if (pre[w]<0) {
                parent[w]=v; pre[w]=low[w]=int(dfs.size()); dfs.push_back(w); stack.push_back(w);
            } else low[v]=std::min(low[v],pre[w]);
        } else {
            stack.pop_back(); if (parent[v]>=0) low[parent[v]]=std::min(low[parent[v]],low[v]);
        }
    }
    if (int(dfs.size())!=n) return std::nullopt;
    // Sign of the lowpoint determines on which side of the DFS parent to insert.
    std::list<int> order{s,t}; std::vector<std::list<int>::iterator> position(n);
    position[s]=order.begin(); position[t]=std::next(order.begin());
    std::vector<bool> before(n); before[s]=true;
    for (int i=2; i<n; ++i) {
        int v=dfs[i],p=parent[v]; auto at=position[p];
        bool left=before[dfs[low[v]]];
        if (!left) ++at;
        position[v]=order.insert(at,v); before[p]=!left;
    }
    STNumberingResult r; r.order.assign(order.begin(),order.end()); r.rank.resize(n);
    if (r.order.front()!=s || r.order.back()!=t) return std::nullopt;
    for (int i=0; i<n; ++i) r.rank[r.order[i]]=i;
    for (int v=0; v<n; ++v) {
        bool lower=v==s,higher=v==t;
        for (auto e:g[v]) {lower=lower || r.rank[e.to]<r.rank[v]; higher=higher || r.rank[e.to]>r.rank[v];}
        if (!lower || !higher) return std::nullopt;
    }
    for (auto e:g.edges()) {
        if (r.rank[e.from]>r.rank[e.to]) std::swap(e.from,e.to);
        r.oriented_edges.emplace_back(e.from,e.to);
    }
    return r;
}
} // namespace cp
