#pragma once
#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <vector>
#include "graph.hpp"
#include "../union_find/disjoint_set_union.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"

namespace cp {
template<class T> struct BoruvkaEdge { int from,to; T cost; int id=-1; };
struct BoruvkaComponents { std::vector<int> id; std::vector<std::vector<int>> vertices; };
template<class T> struct BoruvkaResult { OptimizationWide cost=0; int components=0; std::vector<BoruvkaEdge<T>> edges; };
namespace detail {
template<class T> bool boruvka_less(const BoruvkaEdge<T>& a,const BoruvkaEdge<T>& b) {
    return std::tie(a.cost,a.from,a.to,a.id)<std::tie(b.cost,b.from,b.to,b.id);
}
}
// oracle(partition) returns one minimum outgoing edge (or nullopt) per component.
// The oracle must use the same fixed undirected graph on every round.
template<class T, class Oracle> BoruvkaResult<T> boruvka_msf(int n,Oracle oracle) {
    static_assert(std::is_integral_v<T> && sizeof(T)<=8 && !std::is_same_v<T,bool>);
    if (n<0) throw std::invalid_argument("negative Boruvka size");
    DisjointSetUnion dsu(n); BoruvkaResult<T> r;
    while (dsu.component_count()>1) {
        BoruvkaComponents p; p.vertices=dsu.groups(); p.id.resize(n);
        for (int i=0; i<int(p.vertices.size()); ++i) for (int v:p.vertices[i]) p.id[v]=i;
        auto candidates=oracle(p);
        if (candidates.size()!=p.vertices.size()) throw std::invalid_argument("Boruvka oracle result size");
        // Validate against the partition before merging any candidates.
        for (int i=0; i<int(candidates.size()); ++i) if (candidates[i]) {
            auto e=*candidates[i];
            if (e.from<0 || e.to<0 || e.from>=n || e.to>=n || p.id[e.from]==p.id[e.to]
                    || (p.id[e.from]!=i && p.id[e.to]!=i)) throw std::invalid_argument("Boruvka oracle edge");
        }
        bool changed=false;
        for (const auto& candidate:candidates) if (candidate) {
            auto e=*candidate;
            if (dsu.merge(e.from,e.to)) {
                r.edges.push_back(e);r.cost=optimization_detail::add(r.cost,OptimizationWide(e.cost));changed=true;
            }
        }
        if (!changed) break;
    }
    r.components=dsu.component_count();return r;
}
template<class T> BoruvkaResult<T> boruvka_msf(const Graph<T>& g) {
    if (g.directed()) throw std::invalid_argument("Boruvka requires undirected graph");
    return boruvka_msf<T>(g.size(),[&](const BoruvkaComponents& p) {
        std::vector<std::optional<BoruvkaEdge<T>>> candidates(p.vertices.size());
        for (auto e:g.edges()) if (p.id[e.from]!=p.id[e.to]) {
            BoruvkaEdge<T> edge{e.from,e.to,e.cost,e.idx};
            for (int i:{p.id[e.from],p.id[e.to]}) if (!candidates[i] || detail::boruvka_less(edge,*candidates[i])) candidates[i]=edge;
        }
        return candidates;
    });
}
// Complete graph cost(u,v)=values[u] xor values[v]. O(64 n log n), O(64 n).
inline BoruvkaResult<std::uint64_t> xor_minimum_spanning_tree(const std::vector<std::uint64_t>& values) {
    using E=BoruvkaEdge<std::uint64_t>; int n=int(values.size());
    return boruvka_msf<std::uint64_t>(n,[&](const BoruvkaComponents& p) {
        struct Node { int child[2]={-1,-1}; int first=-1,second=-1; };
        std::vector<Node> trie(1);
        auto record=[&](int k,int v) {
            if (trie[k].first<0) trie[k].first=v;
            else if (p.id[trie[k].first]!=p.id[v] && trie[k].second<0) trie[k].second=v;
        };
        for (int v=0; v<n; ++v) {
            int k=0;record(k,v);
            for (int bit=63; bit>=0; --bit) {
                int b=int((values[v]>>bit)&1);
                if (trie[k].child[b]<0) {int next=int(trie.size());trie[k].child[b]=next;trie.emplace_back();}
                k=trie[k].child[b];record(k,v);
            }
        }
        std::vector<std::optional<E>> candidates(p.vertices.size());
        for (int v=0; v<n; ++v) {
            auto available=[&](int k) {return k>=0 && (p.id[trie[k].first]!=p.id[v] || trie[k].second>=0);};
            int k=0;
            if (!available(k)) continue;
            for (int bit=63; bit>=0; --bit) {
                int b=int((values[v]>>bit)&1);
                if (!available(trie[k].child[b])) b^=1;
                k=trie[k].child[b];
            }
            int u=trie[k].first; if (p.id[u]==p.id[v]) u=trie[k].second;
            E e{std::min(u,v),std::max(u,v),values[u]^values[v],-1};auto& best=candidates[p.id[v]];
            if (!best || detail::boruvka_less(e,*best)) best=e;
        }
        return candidates;
    });
}
} // namespace cp
