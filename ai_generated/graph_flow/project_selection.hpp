#pragma once
#include <cassert>
#include <limits>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "dinic.hpp"
#include "detail/checked_capacity.hpp"

namespace cp {
template<class Cost> struct ProjectSelectionResult { Cost cost; std::vector<bool> selected; };
// 二値変数の最小費用。true=source側。罰金、強制/含意、submodular二変数費用。
template<class Cost=long long> class ProjectSelection {
    static_assert(std::is_integral<Cost>::value && std::is_signed<Cost>::value,"signed integer costs required");
    struct Penalty { int from,to; Cost cost; };
    std::vector<Cost> zero_,one_; std::vector<Penalty> penalties_,hard_; Cost constant_=0;
    void check(int v) const { assert(0<=v && v<size()); (void)v; }
public:
    explicit ProjectSelection(int n):zero_(n,0),one_(n,0) { assert(0<=n && n<=100000000); }
    int size() const { return int(zero_.size()); }
    void add_cost(int v,Cost when_false,Cost when_true) {
        check(v); Cost a=detail::bounded_flow_add(zero_[v],when_false),b=detail::bounded_flow_add(one_[v],when_true);
        zero_[v]=a; one_[v]=b;
    }
    void add_profit(int v,Cost profit) { add_cost(v,0,detail::bounded_flow_subtract(Cost(0),profit)); }
    void add_penalty(int from,int to,Cost cost) {
        check(from); check(to); assert(cost>=0); if(from!=to && cost) penalties_.push_back({from,to,cost});
    }
    void imply(int from,int to) { check(from); check(to); if(from!=to) hard_.push_back({from,to,0}); }
    void force(int v,bool selected) { check(v); hard_.push_back({selected?size():v,selected?v:size()+1,0}); }
    // 費用の添字は(x_u,x_v)。e00+e11<=e01+e10が条件。
    void add_pairwise(int u,int v,Cost e00,Cost e01,Cost e10,Cost e11) {
        check(u); check(v); if(u==v) { add_cost(u,e00,e11); return; }
        Cost a=detail::bounded_flow_subtract(e11,e01),b=detail::bounded_flow_subtract(e01,e00);
        Cost c=detail::bounded_flow_subtract(detail::bounded_flow_subtract(e10,e00),a);
        if(c<0) throw std::invalid_argument("pairwise cost is not submodular");
        Cost base=detail::bounded_flow_add(constant_,e00),x=detail::bounded_flow_add(one_[u],a),y=detail::bounded_flow_add(one_[v],b);
        constant_=base; one_[u]=x; one_[v]=y; add_penalty(u,v,c);
    }
    std::optional<ProjectSelectionResult<Cost>> solve() const {
        int n=size(),s=n,t=n+1; Dinic<Cost> net(n+2); Cost base=constant_,soft=0;
        auto edge=[&](int u,int v,Cost c) { net.add_edge(u,v,c); soft=detail::bounded_flow_add(soft,c); };
        for(int v=0;v<n;++v) {
            if(zero_[v]<=one_[v]) { base=detail::bounded_flow_add(base,zero_[v]); edge(v,t,detail::bounded_flow_subtract(one_[v],zero_[v])); }
            else { base=detail::bounded_flow_add(base,one_[v]); edge(s,v,detail::bounded_flow_subtract(zero_[v],one_[v])); }
        }
        for(const auto& p:penalties_) edge(p.from,p.to,p.cost);
        Cost limit=soft;
        if(!hard_.empty()) {
            limit=detail::bounded_flow_add(soft,Cost(1));
            for(const auto& p:hard_) net.add_edge(p.from,p.to,limit);
        }
        Cost flow=net.max_flow(s,t,limit);
        if(!hard_.empty() && flow==limit) return std::nullopt;
        auto side=net.min_cut(s); side.resize(n);
        return ProjectSelectionResult<Cost>{detail::bounded_flow_add(base,flow),std::move(side)};
    }
};
} // namespace cp
