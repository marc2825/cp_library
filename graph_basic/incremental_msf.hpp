#pragma once
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "../tree/link_cut_tree.hpp"
#include "../combinatorial_optimization/detail/arithmetic.hpp"

namespace cp {
namespace detail {
struct IncrementalMSFValue { OptimizationWide cost=0; int edge=-1; };
inline IncrementalMSFValue incremental_msf_identity() {return {};}
inline IncrementalMSFValue incremental_msf_max(IncrementalMSFValue a,IncrementalMSFValue b) {
    if (a.edge<0) return b;
    if (b.edge<0) return a;
    return a.cost>b.cost || (a.cost==b.cost && a.edge>b.edge)?a:b;
}
}
// Online insertions, amortized O(log n). Equal-weight existing edges are retained.
template<class T=long long> class IncrementalMSF {
    static_assert(std::is_integral_v<T> && sizeof(T)<=8 && !std::is_same_v<T,bool>);
public:
    struct EdgeInfo {int from,to;T cost;bool selected=false;int node=-1;};
    struct Insertion {int id;bool selected;int removed=-1;};
private:
    int n_,components_;
    LinkCutTree<detail::IncrementalMSFValue,detail::incremental_msf_max,detail::incremental_msf_identity> tree_;
    std::vector<EdgeInfo> edges_;
    OptimizationWide cost_=0;
    static int checked(int n) {if (n<0) throw std::invalid_argument("negative MSF size");return n;}
public:
    explicit IncrementalMSF(int n):n_(checked(n)),components_(n_),tree_(n_){}
    int size() const {return n_;}
    int components() const {return components_;}
    OptimizationWide cost() const {return cost_;}
    const std::vector<EdgeInfo>& edges() const {return edges_;}
    std::vector<int> selected_edges() const {
        std::vector<int> r;for (int i=0;i<int(edges_.size());++i) if (edges_[i].selected) r.push_back(i);return r;
    }
    Insertion add_edge(int u,int v,T weight) {
        if (u<0 || v<0 || u>=n_ || v>=n_) throw std::out_of_range("MSF vertex");
        int id=int(edges_.size());edges_.push_back({u,v,weight,false,-1});
        Insertion result{id,false,-1}; if (u==v) return result;
        int node=-1;
        if (tree_.connected(u,v)) {
            auto largest=*tree_.path_prod(u,v);
            if (OptimizationWide(weight)>=largest.cost) return result;
            result.removed=largest.edge;auto& old=edges_[largest.edge];node=old.node;
            tree_.cut(old.from,node);tree_.cut(node,old.to);old.selected=false;old.node=-1;
            cost_=optimization_detail::sub(cost_,largest.cost);tree_.set(node,{OptimizationWide(weight),id});
        } else {node=tree_.add_vertex({OptimizationWide(weight),id});--components_;}
        tree_.link(u,node);tree_.link(node,v);
        cost_=optimization_detail::add(cost_,OptimizationWide(weight));
        edges_[id].selected=true;edges_[id].node=node;result.selected=true;return result;
    }
};
} // namespace cp
