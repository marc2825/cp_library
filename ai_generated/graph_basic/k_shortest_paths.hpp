#pragma once
#include <algorithm>
#include <cassert>
#include <functional>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>
#include "graph.hpp"
#include "shortest_path.hpp"
#include "weighted_path.hpp"

namespace cp {
// Yen法。頂点重複なしの経路をコスト昇順に高々k個。多重辺は別経路。
template<class T> std::vector<WeightedPath<T>> k_shortest_paths(const Graph<T>& g,int s,int t,int k) {
    static_assert(std::is_integral<T>::value && !std::is_same<T,bool>::value,"integer weights required");
    assert(0<=s && s<g.size() && 0<=t && t<g.size() && k>=0);
    for(const auto& e:g.edges()) { assert(e.cost>=T(0)); (void)e; }
    std::vector<WeightedPath<T>> answer; if(!k) return answer;
    struct TrieNode { std::map<int,int> child; }; std::vector<TrieNode> trie(1);
    std::vector<bool> banned_vertex(g.size(),false);
    std::vector<std::size_t> banned_edge(g.edge_count(),0); std::size_t stamp=1;
    auto shortest=[&](int start)->std::optional<WeightedPath<T>> {
        ShortestPathResult<T> paths(g.size(),start); paths.dist[start]=T(0);
        using Entry=std::pair<T,int>; std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
        queue.emplace(T(0),start);
        while(!queue.empty()) {
            auto [cost,v]=queue.top(); queue.pop(); if(cost!=paths.dist[v]) continue; if(v==t) break;
            for(const auto& e:g[v]) {
                if(banned_vertex[e.to] || banned_edge[e.idx]==stamp) continue;
                T next=detail::distance_add(cost,e.cost); if(next>=paths.dist[e.to]) continue;
                paths.dist[e.to]=next; paths.parent[e.to]=v; paths.parent_edge[e.to]=e.idx; queue.emplace(next,e.to);
            }
        }
        if(!paths.reachable(t)) return std::nullopt;
        return WeightedPath<T>{paths.dist[t],paths.vertex_path(t),paths.edge_path(t)};
    };
    auto first=shortest(s); if(!first) return answer; answer.push_back(std::move(*first));
    auto register_path=[&](const WeightedPath<T>& path) {
        int node=0;
        for(int edge:path.edge_ids) {
            auto found=trie[node].child.find(edge);
            if(found!=trie[node].child.end()) node=found->second;
            else { int child=int(trie.size()); trie[node].child.emplace(edge,child); trie.push_back({}); node=child; }
        }
    };
    register_path(answer.back());
    auto less=[](const WeightedPath<T>& a,const WeightedPath<T>& b) { return a.cost!=b.cost?a.cost<b.cost:a.edge_ids<b.edge_ids; };
    std::set<WeightedPath<T>,decltype(less)> candidates(less);
    while(int(answer.size())<k) {
        const auto& last=answer.back(); T prefix_cost=T(0); int node=0;
        std::fill(banned_vertex.begin(),banned_vertex.end(),false);
        for(int i=0;i<int(last.edge_ids.size());++i) {
            ++stamp;
            for(const auto& child:trie[node].child) banned_edge[child.first]=stamp;
            auto suffix=shortest(last.vertices[i]);
            if(suffix) {
                WeightedPath<T> path{detail::distance_add(prefix_cost,suffix->cost),
                    std::vector<int>(last.vertices.begin(),last.vertices.begin()+i),
                    std::vector<int>(last.edge_ids.begin(),last.edge_ids.begin()+i)};
                path.vertices.insert(path.vertices.end(),suffix->vertices.begin(),suffix->vertices.end());
                path.edge_ids.insert(path.edge_ids.end(),suffix->edge_ids.begin(),suffix->edge_ids.end());
                candidates.insert(std::move(path));
            }
            banned_vertex[last.vertices[i]]=true;
            prefix_cost=detail::distance_add(prefix_cost,g.edges()[last.edge_ids[i]].cost);
            node=trie[node].child.at(last.edge_ids[i]);
        }
        if(candidates.empty()) break;
        answer.push_back(*candidates.begin()); candidates.erase(candidates.begin()); register_path(answer.back());
    }
    return answer;
}
} // namespace cp
