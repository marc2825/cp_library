#pragma once
#include <cassert>
#include <limits>
#include <optional>
#include <type_traits>
#include <vector>
#include "dinic.hpp"
#include "detail/checked_capacity.hpp"

namespace cp {
template<class Cap> struct BoundedFlowResult { Cap value; std::vector<Cap> flow; };
// 下限<=flow<=上限。各solveは独立に再計算する。valueは負のs-t流も可。
template<class Cap=long long> class LowerBoundFlow {
    static_assert(std::is_integral<Cap>::value && std::is_signed<Cap>::value,"signed integer capacity required");
    struct Edge { int from,to; Cap lower,upper; };
    int n_; std::vector<Edge> edges_;
    std::optional<std::vector<Cap>> feasible(int s,int t) const {
        Dinic<Cap> net(n_+2); std::vector<Cap> balance(n_,Cap(0));
        for(const auto& e:edges_) {
            net.add_edge(e.from,e.to,e.upper-e.lower);
            if(e.from!=e.to) {
                balance[e.from]=detail::bounded_flow_subtract(balance[e.from],e.lower);
                balance[e.to]=detail::bounded_flow_add(balance[e.to],e.lower);
            }
        }
        if(s!=-1) { net.add_edge(t,s,std::numeric_limits<Cap>::max()); net.add_edge(s,t,std::numeric_limits<Cap>::max()); }
        Cap demand=0;
        for(int v=0;v<n_;++v) {
            if(balance[v]>0) { net.add_edge(n_,v,balance[v]); demand=detail::bounded_flow_add(demand,balance[v]); }
            else if(balance[v]<0) net.add_edge(v,n_+1,detail::bounded_flow_subtract(Cap(0),balance[v]));
        }
        if(net.max_flow(n_,n_+1,demand)!=demand) return std::nullopt;
        std::vector<Cap> flows;
        for(int i=0;i<int(edges_.size());++i) flows.push_back(detail::bounded_flow_add(edges_[i].lower,net.get_edge(i).flow));
        return flows;
    }
    Cap value(const std::vector<Cap>& flow,int s) const {
        Cap result=0;
        for(int i=0;i<int(edges_.size());++i) {
            const auto& e=edges_[i]; if(e.from==e.to) continue;
            if(e.from==s) result=detail::bounded_flow_add(result,flow[i]);
            if(e.to==s) result=detail::bounded_flow_subtract(result,flow[i]);
        }
        return result;
    }
    void endpoints(int s,int t) const { assert(0<=s && s<n_ && 0<=t && t<n_ && s!=t); (void)s; (void)t; }
public:
    explicit LowerBoundFlow(int n):n_(n) { assert(0<=n && n<=100000000); }
    int add_edge(int from,int to,Cap lower,Cap upper) {
        assert(0<=from && from<n_ && 0<=to && to<n_ && Cap(0)<=lower && lower<=upper);
        edges_.push_back({from,to,lower,upper}); return int(edges_.size())-1;
    }
    int edge_count() const { return int(edges_.size()); }
    std::optional<std::vector<Cap>> feasible_circulation() const { return feasible(-1,-1); }
    std::optional<BoundedFlowResult<Cap>> feasible_flow(int s,int t) const {
        endpoints(s,t); auto f=feasible(s,t); if(!f) return std::nullopt;
        return BoundedFlowResult<Cap>{value(*f,s),std::move(*f)};
    }
    std::optional<BoundedFlowResult<Cap>> maximum_flow(int s,int t) const {
        endpoints(s,t); auto f=feasible(s,t); if(!f) return std::nullopt;
        Cap total=value(*f,s); Dinic<Cap> residual(n_);
        for(int i=0;i<int(edges_.size());++i) {
            const auto& e=edges_[i]; residual.add_edge(e.from,e.to,e.upper-(*f)[i]); residual.add_edge(e.to,e.from,(*f)[i]-e.lower);
        }
        for(;;) {
            Cap limit=total<0?std::numeric_limits<Cap>::max():std::numeric_limits<Cap>::max()-total;
            Cap added=residual.max_flow(s,t,limit); total=detail::bounded_flow_add(total,added);
            if(added<limit) break;
            if(total==std::numeric_limits<Cap>::max()) {
                if(residual.min_cut(s)[t]) throw std::overflow_error("bounded maximum flow overflow");
                break;
            }
        }
        for(int i=0;i<int(edges_.size());++i) {
            Cap a=residual.get_edge(2*i).flow,b=residual.get_edge(2*i+1).flow;
            (*f)[i]=a>=b?detail::bounded_flow_add((*f)[i],a-b):detail::bounded_flow_subtract((*f)[i],b-a);
        }
        return BoundedFlowResult<Cap>{total,std::move(*f)};
    }
};
} // namespace cp
