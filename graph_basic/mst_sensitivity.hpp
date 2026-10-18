#pragma once
// 説明: docs/graph_basic/mst_sensitivity.md
#include <array>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "kruskal.hpp"
#include "../tree/heavy_light_decomposition.hpp"
#include "../range_query/segtree.hpp"
#include "../range_query/dual_segtree.hpp"

namespace cp {
template<class T> struct SpanningTreeExchange { T cost; int added_edge,removed_edge; };
template<class T> struct MSTSensitivityResult {
    T cost=T(0);
    std::vector<bool> in_tree;
    std::vector<std::optional<T>> forced_cost,forbidden_cost;
    std::vector<int> replacement_edge,removed_edge;
    std::optional<SpanningTreeExchange<T>> different,strictly_larger;
};
namespace detail {
template<class T> struct MSTPathMonoid {
    using Item=std::pair<T,int>;
    using S=std::array<std::optional<Item>,2>;
    static S e() { return {}; }
    static S op(S a,S b) {
        S result{};
        for(const auto& item:{a[0],a[1],b[0],b[1]}) if(item) {
            if(!result[0] || result[0]->first<item->first) { result[1]=result[0]; result[0]=item; }
            else if(item->first<result[0]->first && (!result[1] || result[1]->first<item->first)) result[1]=item;
        }
        return result;
    }
    using F=std::optional<Item>;
    static F id() { return std::nullopt; }
    static F minimum(F a,F b) { if(!a) return b; if(!b) return a; return *a<*b?a:b; }
};
}
// 連結無向グラフ。辺の強制/禁止と同重みを含む別MST、厳密に高い次の木。
template<class T> std::optional<MSTSensitivityResult<T>> mst_sensitivity(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("MST sensitivity requires undirected graph");
    auto mst=kruskal(g); if(!mst.connected()) return std::nullopt;
    MSTSensitivityResult<T> answer; answer.cost=mst.cost;
    const int n=g.size(),m=g.edge_count(); answer.in_tree.assign(m,false); answer.forced_cost.resize(m); answer.forbidden_cost.assign(m,mst.cost);
    answer.replacement_edge.assign(m,-1); answer.removed_edge.assign(m,-1);
    if(!n) return answer;
    Graph<T> tree(n); std::vector<int> original;
    for(const auto& e:mst.edges) { answer.in_tree[e.idx]=true; tree.add_edge(e.from,e.to,e.cost); original.push_back(e.idx); answer.forced_cost[e.idx]=mst.cost; answer.forbidden_cost[e.idx]=std::nullopt; }
    HeavyLightDecomposition<T> hld(tree); using M=detail::MSTPathMonoid<T>; using S=typename M::S; using F=typename M::F;
    std::vector<S> values(n,M::e());
    for(int v=0;v<n;++v) if(hld.tree.parent_edge[v]>=0) {
        int id=original[hld.tree.parent_edge[v]]; values[hld.pos[v]][0]=std::make_pair(g.edges()[id].cost,id);
    }
    Segtree<S,M::op,M::e> maximum(values);
    DualSegtree<F,F,M::minimum,M::minimum,M::id> minimum(n,M::id());
    auto exchange_cost=[&](T added,T removed) { return detail::distance_add(mst.cost,detail::distance_subtract(added,removed)); };
    auto improve=[](auto& slot,T cost,int added,int removed) { if(!slot || cost<slot->cost) slot=SpanningTreeExchange<T>{cost,added,removed}; };
    for(const auto& edge:g.edges()) if(!answer.in_tree[edge.idx] && edge.from!=edge.to) {
        S best=M::e();
        for(const auto& segment:hld.path_segments(edge.from,edge.to,true)) {
            best=M::op(best,maximum.prod(segment.l,segment.r)); minimum.apply(segment.l,segment.r,std::make_pair(edge.cost,edge.idx));
        }
        const auto top=*best[0]; T cost=exchange_cost(edge.cost,top.first);
        answer.forced_cost[edge.idx]=cost; answer.removed_edge[edge.idx]=top.second;
        improve(answer.different,cost,edge.idx,top.second);
        if(mst.cost<cost) improve(answer.strictly_larger,cost,edge.idx,top.second);
        else if(best[1]) improve(answer.strictly_larger,exchange_cost(edge.cost,best[1]->first),edge.idx,best[1]->second);
    }
    for(int v=0;v<n;++v) if(hld.tree.parent_edge[v]>=0) {
        int id=original[hld.tree.parent_edge[v]]; auto alternative=minimum.get(hld.pos[v]);
        if(alternative) { answer.forbidden_cost[id]=exchange_cost(alternative->first,g.edges()[id].cost); answer.replacement_edge[id]=alternative->second; }
    }
    return answer;
}
} // namespace cp
