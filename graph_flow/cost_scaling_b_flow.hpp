#pragma once
// 説明: docs/graph_flow/cost_scaling_b_flow.md
#include <algorithm>
#include <functional>
#include <optional>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "../combinatorial_optimization/detail/arithmetic.hpp"
namespace cp {
// 整数 epsilon-scaling。正の balance は流出超過。solve は入力を変更しない。
template<class Cap=long long,class Cost=long long> class CostScalingBFlow {
    static_assert(std::is_integral_v<Cap> && std::is_signed_v<Cap> && sizeof(Cap)<=8);
    static_assert(std::is_integral_v<Cost> && std::is_signed_v<Cost> && sizeof(Cost)<=8);
    using W=OptimizationWide;
    struct Edge { int from,to; Cap lower,upper; Cost cost; };
    int n_; std::vector<Cap> balance_; std::vector<Edge> edges_;
    void vertex(int v) const { if(v<0 || v>=n_) throw std::out_of_range("flow vertex"); }
public:
    struct Result { W cost=0; std::vector<Cap> flow; std::vector<W> potential; };
    explicit CostScalingBFlow(int n):n_(n) { if(n<0) throw std::invalid_argument("negative size"); balance_.resize(n); }
    int edge_count() const { return int(edges_.size()); }
    void set_balance(int v,Cap b) { vertex(v); balance_[v]=b; }
    int add_edge(int u,int v,Cap lower,Cap upper,Cost cost) {
        vertex(u); vertex(v); if(lower>upper) throw std::invalid_argument("lower exceeds upper");
        edges_.push_back({u,v,lower,upper,cost}); return edge_count()-1;
    }
    std::optional<Result> solve() const {
        using namespace optimization_detail;
        W sum=0; for(Cap b:balance_) sum=add(sum,W(b)); if(sum!=0) return std::nullopt;
        struct Arc { int from,to; W cap,cost; };
        int N=n_+2,source=n_,sink=n_+1; std::vector<Arc> arcs; std::vector<std::vector<int>> adj(N);
        auto insert=[&](int u,int v,W cap,W cost) {
            int id=int(arcs.size()); arcs.push_back({u,v,cap,cost}); arcs.push_back({v,u,0,-cost});
            adj[u].push_back(id); adj[v].push_back(id+1);
        };
        std::vector<W> need(balance_.begin(),balance_.end()); W epsilon=1;
        for(auto e:edges_) {
            W c=mul(W(e.cost),W(n_)+1); insert(e.from,e.to,W(e.upper)-e.lower,c);
            need[e.from]=sub(need[e.from],e.lower); need[e.to]=add(need[e.to],e.lower);
            epsilon=std::max(epsilon,c<0?-c:c);
        }
        int real=int(arcs.size()); W demand=0;
        for(int v=0;v<n_;++v) {
            if(need[v]>0) { insert(source,v,need[v],0); demand=add(demand,need[v]); }
            if(need[v]<0) insert(v,sink,-need[v],0);
        }
        auto push=[&](int id,W f) { arcs[id].cap-=f; arcs[id^1].cap+=f; };
        std::vector<int> level(N),cur(N);
        std::function<W(int,W)> dfs=[&](int v,W limit)->W {
            if(v==sink) return limit;
            for(int& j=cur[v];j<int(adj[v].size());++j) {
                int id=adj[v][j]; auto e=arcs[id];
                if(e.cap>0 && level[e.to]==level[v]+1) {
                    W sent=dfs(e.to,std::min(limit,e.cap)); if(sent) { push(id,sent); return sent; }
                }
            }
            return 0;
        };
        while(demand>0) {
            std::fill(level.begin(),level.end(),-1); std::queue<int> q; q.push(source); level[source]=0;
            while(!q.empty()) { int v=q.front(); q.pop(); for(int id:adj[v]) if(arcs[id].cap>0 && level[arcs[id].to]<0) { level[arcs[id].to]=level[v]+1; q.push(arcs[id].to); } }
            if(level[sink]<0) return std::nullopt;
            std::fill(cur.begin(),cur.end(),0); while(W sent=dfs(source,demand)) demand-=sent;
        }
        arcs.resize(real); adj.resize(n_);
        for(auto& a:adj) a.erase(std::remove_if(a.begin(),a.end(),[&](int id){return id>=real;}),a.end());
        std::vector<W> potential(n_),excess(n_); std::vector<bool> active(n_);
        auto reduced=[&](int id) { auto e=arcs[id]; return add(e.cost,sub(potential[e.from],potential[e.to])); };
        do {
            epsilon=std::max(W(1),(epsilon+1)/2);
            std::fill(excess.begin(),excess.end(),0);
            for(int id=0;id<real;++id) if(arcs[id].cap>0 && reduced(id)<0) {
                W f=arcs[id].cap; excess[arcs[id].from]-=f; excess[arcs[id].to]+=f; push(id,f);
            }
            std::queue<int> q;
            for(int v=0;v<n_;++v) if(excess[v]>0) { q.push(v); active[v]=true; }
            std::fill(cur.begin(),cur.end(),0);
            while(!q.empty()) {
                int v=q.front(); q.pop(); active[v]=false;
                while(excess[v]>0) {
                    if(cur[v]==int(adj[v].size())) {
                        std::optional<W> delta;
                        for(int id:adj[v]) if(arcs[id].cap>0) { W d=reduced(id); if(!delta || d<*delta) delta=d; }
                        if(!delta) throw std::logic_error("infeasible residual excess");
                        potential[v]=sub(potential[v],add(*delta,epsilon)); cur[v]=0;
                    } else {
                        int id=adj[v][cur[v]],u=arcs[id].to;
                        if(arcs[id].cap>0 && reduced(id)<0) {
                            W f=std::min(excess[v],arcs[id].cap); push(id,f); excess[v]-=f; excess[u]+=f;
                            if(u!=v && excess[u]>0 && !active[u]) { active[u]=true; q.push(u); }
                        } else ++cur[v];
                    }
                }
            }
        } while(epsilon>1);
        Result answer; answer.potential.assign(n_,0);
        for(int i=0;i<edge_count();++i) {
            W f=add(W(edges_[i].lower),arcs[2*i+1].cap); answer.flow.push_back(narrow<Cap>(f));
            answer.cost=add(answer.cost,mul(f,W(edges_[i].cost)));
        }
        // 元の費用単位の双対証明。epsilon=1 と n+1 倍により負閉路はない。
        for(int pass=0;pass<n_;++pass) {
            bool changed=false;
            for(int id=0;id<real;++id) if(arcs[id].cap>0) {
                auto e=arcs[id]; W d=add(answer.potential[e.from],e.cost/(W(n_)+1));
                if(d<answer.potential[e.to]) { answer.potential[e.to]=d; changed=true; }
            }
            if(!changed) break;
            if(pass==n_-1) throw std::logic_error("cost scaling negative cycle");
        }
        return answer;
    }
};
} // namespace cp
