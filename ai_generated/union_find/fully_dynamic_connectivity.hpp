#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>
#include "../tree/euler_tour_tree.hpp"
namespace cp {
namespace detail {
inline int connectivity_sum(int a,int b){return a+b;}
inline int connectivity_zero(){return 0;}
}
// オンライン無向多重グラフ。森辺の削除は小さい側の代替辺を探索する。
// HDTの多重対数更新時間を保証する実装ではない。docs/union_find/fully_dynamic_connectivity.md
class FullyDynamicConnectivity {
    struct Edge {int u,v;bool active=true,tree=false;};
    using Forest=EulerTourTree<int,detail::connectivity_sum,detail::connectivity_zero>;
    Forest forest_;
    std::vector<Edge> edges_;
    std::vector<std::set<int>> incident_,tree_edges_;
    std::vector<std::uint64_t> seen_;
    std::uint64_t epoch_=0;
    int components_;
    void check(int v) const {assert(0<=v && v<size());(void)v;}
    void adopt(int id) {
        auto& e=edges_[id];bool linked=forest_.link(e.u,e.v);assert(linked);(void)linked;
        e.tree=true;tree_edges_[e.u].insert(id);tree_edges_[e.v].insert(id);--components_;
    }
public:
    explicit FullyDynamicConnectivity(int n) : forest_(n),incident_(n),tree_edges_(n),seen_(n),components_(n) {}
    int size() const {return forest_.size();}
    int component_count() const {return components_;}
    int component_size(int v) const {check(v);return forest_.component_size(v);}
    bool connected(int u,int v) const {check(u);check(v);return forest_.connected(u,v);}
    int add_edge(int u,int v) {
        check(u);check(v);int id=int(edges_.size());edges_.push_back({u,v});
        incident_[u].insert(id);incident_[v].insert(id);
        if(!forest_.connected(u,v)) adopt(id);
        return id;
    }
    bool active(int id) const {return 0<=id && id<int(edges_.size()) && edges_[id].active;}
    bool remove_edge(int id) {
        if(!active(id)) return false;
        auto& e=edges_[id];e.active=false;incident_[e.u].erase(id);incident_[e.v].erase(id);
        if(!e.tree) return true;
        e.tree=false;forest_.cut(e.u,e.v);tree_edges_[e.u].erase(id);tree_edges_[e.v].erase(id);++components_;
        int start=forest_.component_size(e.u)<=forest_.component_size(e.v)?e.u:e.v;
        if(++epoch_==0) {std::fill(seen_.begin(),seen_.end(),0);++epoch_;}
        std::vector<int> vertices{start};seen_[start]=epoch_;
        for(std::size_t i=0;i<vertices.size();++i) for(int edge:tree_edges_[vertices[i]]) {
            const auto& arc=edges_[edge];int next=arc.u^arc.v^vertices[i];
            if(seen_[next]!=epoch_) {seen_[next]=epoch_;vertices.push_back(next);}
        }
        for(int v:vertices) for(int edge:incident_[v]) {
            const auto& arc=edges_[edge];int next=arc.u^arc.v^v;
            if(seen_[next]!=epoch_) {adopt(edge);return true;}
        }
        return true;
    }
    // IDは保持し、過去に切断されたETT tokenだけを回収する。
    void compact_forest() {
        Forest next(size());
        for(const auto& e:edges_) if(e.active && e.tree) next.link(e.u,e.v);
        forest_=std::move(next);
    }
};
} // namespace cp
