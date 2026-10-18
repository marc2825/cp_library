#pragma once
// 説明: docs/graph_flow/min_cost_b_flow.md
#include <algorithm>
#include <cassert>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include "../combinatorial_optimization/detail/arithmetic.hpp"

namespace cp {
// net outflow b[v]を指定。負費用辺を先に上限まで飽和し、非負残余費用からSSP。
template<class Cap=long long,class Cost=long long> class MinCostBFlow {
    static_assert(std::is_integral<Cap>::value && std::is_signed<Cap>::value && sizeof(Cap)<=8,"signed capacity <=64bit required");
    static_assert(std::is_integral<Cost>::value && std::is_signed<Cost>::value && sizeof(Cost)<=8,"signed cost <=64bit required");
    using W=OptimizationWide;
    struct Edge { int from,to; Cap lower,upper; Cost cost; };
    int n_; std::vector<Edge> edges_; std::vector<Cap> balance_;
    void vertex(int v) const { assert(0<=v && v<n_); (void)v; }
public:
    struct Result { W cost=0; std::vector<Cap> flow; std::vector<W> potential; };
    explicit MinCostBFlow(int n) : n_(n),balance_(n,0) { assert(n>=0); }
    void set_balance(int v,Cap net_outflow) { vertex(v); balance_[v]=net_outflow; }
    int add_edge(int from,int to,Cap lower,Cap upper,Cost cost) {
        vertex(from); vertex(to); if(upper<lower) throw std::invalid_argument("lower exceeds upper");
        edges_.push_back({from,to,lower,upper,cost}); return int(edges_.size())-1;
    }
    int edge_count() const { return int(edges_.size()); }
    std::optional<Result> solve() const {
        using namespace optimization_detail;
        W total=0; for(Cap x:balance_) total=add(total,W(x)); if(total) return std::nullopt;
        struct Arc { int from,to; W residual,cost; };
        int source=n_,sink=n_+1,vertices=n_+2; std::vector<Arc> arcs; std::vector<std::vector<int>> adjacency(vertices);
        auto add_arc=[&](int a,int b,W capacity,W cost) {
            int id=int(arcs.size()); arcs.push_back({a,b,capacity,cost}); arcs.push_back({b,a,0,-cost}); adjacency[a].push_back(id); adjacency[b].push_back(id+1); return id;
        };
        std::vector<W> needed(balance_.begin(),balance_.end()),initial;
        std::vector<int> ids;
        for(const auto& e:edges_) {
            bool negative=e.cost<0; W f=negative?W(e.upper):W(e.lower); initial.push_back(f);
            if(e.from!=e.to) { needed[e.from]=sub(needed[e.from],f); needed[e.to]=add(needed[e.to],f); }
            ids.push_back(negative?add_arc(e.to,e.from,W(e.upper)-W(e.lower),-W(e.cost)):add_arc(e.from,e.to,W(e.upper)-W(e.lower),W(e.cost)));
        }
        W demand=0;
        for(int v=0;v<n_;++v) if(needed[v]>0) { add_arc(source,v,needed[v],0); demand=add(demand,needed[v]); } else if(needed[v]<0) add_arc(v,sink,-needed[v],0);
        std::vector<W> potential(vertices); W sent=0;
        while(sent<demand) {
            std::vector<std::optional<W>> distance(vertices); std::vector<int> parent(vertices,-1);
            using Item=std::pair<W,int>; std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
            distance[source]=0; queue.emplace(0,source);
            while(!queue.empty()) {
                auto [d,v]=queue.top(); queue.pop(); if(d!=*distance[v]) continue;
                for(int id:adjacency[v]) if(arcs[id].residual>0) {
                    const auto& e=arcs[id]; W reduced=add(e.cost,sub(potential[v],potential[e.to]));
                    if(reduced<0) throw std::logic_error("negative reduced cost in b-flow");
                    W next=add(d,reduced);
                    if(!distance[e.to] || next<*distance[e.to]) { distance[e.to]=next; parent[e.to]=id; queue.emplace(next,e.to); }
                }
            }
            if(!distance[sink]) return std::nullopt;
            for(int v=0;v<vertices;++v) if(distance[v]) potential[v]=add(potential[v],*distance[v]);
            W amount=demand-sent;
            for(int v=sink;v!=source;v=arcs[parent[v]].from) amount=std::min(amount,arcs[parent[v]].residual);
            for(int v=sink;v!=source;v=arcs[parent[v]].from) { int id=parent[v]; arcs[id].residual-=amount; arcs[id^1].residual+=amount; }
            sent=add(sent,amount);
        }
        Result answer; answer.flow.resize(edges_.size()); answer.potential.assign(n_,0);
        for(int i=0;i<int(edges_.size());++i) {
            W delta=arcs[ids[i]^1].residual;
            W f=add(initial[i],edges_[i].cost<0?-delta:delta); answer.flow[i]=narrow<Cap>(f); answer.cost=add(answer.cost,mul(f,W(edges_[i].cost)));
        }
        // 全成分に有効な双対証明。到達不能な成分も含めて残余辺の差分制約を解く。
        for(int pass=0;pass<n_;++pass) {
            bool changed=false;
            for(int i=0;i<int(edges_.size());++i) {
                const auto& e=edges_[i];
                auto relax=[&](int a,int b,W cost) { W value=add(answer.potential[a],cost); if(value<answer.potential[b]) { answer.potential[b]=value; changed=true; } };
                if(answer.flow[i]<e.upper) relax(e.from,e.to,W(e.cost));
                if(e.lower<answer.flow[i]) relax(e.to,e.from,-W(e.cost));
            }
            if(!changed) break;
            if(pass==n_-1) throw std::logic_error("negative cycle after b-flow optimization");
        }
        return answer;
    }
};
} // namespace cp
