#pragma once
// 説明: docs/graph_flow/push_relabel.md
#include <algorithm>
#include <cassert>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "../combinatorial_optimization/detail/arithmetic.hpp"

namespace cp {
template<class Cap=long long> class PushRelabel {
    static_assert(std::is_integral<Cap>::value && !std::is_same<Cap,bool>::value && sizeof(Cap)<=8,"integer capacity <=64bit required");
    struct Edge { int from,to; Cap capacity; };
    int n_; std::vector<Edge> edges_;
public:
    struct Result { Cap value; std::vector<Cap> flow; std::vector<bool> source_side; };
    explicit PushRelabel(int n) : n_(n) { assert(n>=0); }
    int add_edge(int from,int to,Cap capacity) {
        assert(0<=from && from<n_ && 0<=to && to<n_);
        if(capacity<Cap(0)) throw std::invalid_argument("negative capacity");
        edges_.push_back({from,to,capacity}); return int(edges_.size())-1;
    }
    Result solve(int source,int sink) const {
        assert(0<=source && source<n_ && 0<=sink && sink<n_ && source!=sink);
        using W=OptimizationWide; struct Arc { int to; Cap residual; };
        std::vector<Arc> arcs; std::vector<std::vector<int>> adjacency(n_);
        for(const auto& e:edges_) { int id=int(arcs.size()); arcs.push_back({e.to,e.capacity}); arcs.push_back({e.from,0}); adjacency[e.from].push_back(id); adjacency[e.to].push_back(id+1); }
        std::vector<W> excess(n_); std::vector<int> height(n_),cursor(n_); std::vector<bool> queued(n_); std::queue<int> active;
        auto enqueue=[&](int v) { if(v!=source && v!=sink && excess[v]>0 && !queued[v]) { queued[v]=true; active.push(v); } };
        height[source]=n_;
        for(int id:adjacency[source]) if(arcs[id].to!=source) {
            Cap amount=arcs[id].residual; arcs[id].residual=0; arcs[id^1].residual+=amount;
            excess[source]-=W(amount); excess[arcs[id].to]+=W(amount); enqueue(arcs[id].to);
        }
        while(!active.empty()) {
            int v=active.front(); active.pop(); queued[v]=false;
            while(excess[v]>0) {
                if(cursor[v]==int(adjacency[v].size())) {
                    int next=2*n_+1;
                    for(int id:adjacency[v]) if(arcs[id].residual>0) next=std::min(next,height[arcs[id].to]+1);
                    if(next>2*n_) throw std::logic_error("stranded preflow");
                    height[v]=next; cursor[v]=0;
                } else {
                    int id=adjacency[v][cursor[v]],u=arcs[id].to;
                    if(arcs[id].residual>0 && height[v]==height[u]+1) {
                        Cap amount=optimization_detail::narrow<Cap>(std::min(excess[v],W(arcs[id].residual)));
                        arcs[id].residual-=amount; arcs[id^1].residual+=amount; excess[v]-=W(amount); excess[u]+=W(amount); enqueue(u);
                    } else ++cursor[v];
                }
            }
        }
        Result result{optimization_detail::narrow<Cap>(excess[sink]),{},std::vector<bool>(n_)};
        for(int i=0;i<int(edges_.size());++i) result.flow.push_back(arcs[2*i+1].residual);
        std::queue<int> queue; queue.push(source); result.source_side[source]=true;
        while(!queue.empty()) { int v=queue.front(); queue.pop(); for(int id:adjacency[v]) if(arcs[id].residual>0 && !result.source_side[arcs[id].to]) { result.source_side[arcs[id].to]=true; queue.push(arcs[id].to); } }
        return result;
    }
    Cap max_flow(int source,int sink) const { return solve(source,sink).value; }
};
} // namespace cp
