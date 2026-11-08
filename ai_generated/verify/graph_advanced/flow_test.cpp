#include "common.hpp"
#include "graph_flow/cost_scaling_b_flow.hpp"
#include "graph_flow/min_cost_b_flow.hpp"
#include "graph_flow/general_weighted_matching.hpp"
#include "graph_flow/rank_maximal_matching.hpp"
#include "graph_flow/dynamic_unit_flow.hpp"
using namespace cp;
int main() {
    std::mt19937 rng(60937);
    for(int trial=0;trial<1000;++trial) {
        int n=1+int(rng()%5),m=int(rng()%8); CostScalingBFlow<> f(n); std::vector<int> balances(n);
        for(int v=0;v<n-1;++v) { balances[v]=int(rng()%5)-2; balances[n-1]-=balances[v]; }
        if(trial%7==0) ++balances.back();
        for(int v=0;v<n;++v) f.set_balance(v,balances[v]);
        struct E { int u,v,lo,hi,c; }; std::vector<E> edges;
        for(int i=0;i<m;++i) { int lo=int(rng()%3)-1; edges.push_back({int(rng()%n),int(rng()%n),lo,lo+int(rng()%3),int(rng()%11)-5}); auto e=edges.back(); f.add_edge(e.u,e.v,e.lo,e.hi,e.c); }
        std::optional<long long> best; std::vector<int> net(n);
        std::function<void(int,long long)> enumerate=[&](int i,long long cost) {
            if(i==m) { if(net==balances && (!best || cost<*best)) best=cost; return; }
            auto e=edges[i]; for(int x=e.lo;x<=e.hi;++x) { net[e.u]+=x; net[e.v]-=x; enumerate(i+1,cost+1LL*x*e.c); net[e.u]-=x; net[e.v]+=x; }
        };
        enumerate(0,0); auto result=f.solve(); CHECK(bool(result)==bool(best));
        if(result) {
            CHECK(result->cost==*best); std::fill(net.begin(),net.end(),0);
            for(int i=0;i<m;++i) { auto e=edges[i]; auto x=result->flow[i]; CHECK(e.lo<=x && x<=e.hi); net[e.u]+=int(x); net[e.v]-=int(x); auto reduced=e.c+result->potential[e.u]-result->potential[e.v]; if(x<e.hi) CHECK(reduced>=0); if(x>e.lo) CHECK(reduced<=0); }
            CHECK(net==balances); CHECK(f.solve()->cost==result->cost);
        }
    }
    for(int trial=0;trial<40;++trial) {
        int n=30; CostScalingBFlow<> f(n); MinCostBFlow<> reference(n);
        for(int i=0;i<250;++i) { int u=int(rng()%n),v=int(rng()%n),c=int(rng()%200)-100; long long cap=1000000000000LL+int(rng()%100); f.add_edge(u,v,0,cap,c); reference.add_edge(u,v,0,cap,c); }
        CHECK(f.solve()->cost==reference.solve()->cost);
    }
    for(int trial=0;trial<2500;++trial) {
        int n=int(rng()%12); Graph<long long> g(n); std::vector<std::vector<long long>> weight(n,std::vector<long long>(n));
        if(n) for(int i=0;i<35;++i) { int u=int(rng()%n),v=int(rng()%n); long long w=int(rng()%31)-10; g.add_edge(u,v,w); if(u!=v) weight[u][v]=weight[v][u]=std::max(weight[u][v],w); }
        std::vector<long long> dp(1<<n);
        for(int mask=1;mask<(1<<n);++mask) { int v=__builtin_ctz(unsigned(mask)),rest=mask^(1<<v); dp[mask]=dp[rest]; for(int u=0;u<n;++u) if(rest>>u&1) dp[mask]=std::max(dp[mask],dp[rest^(1<<u)]+weight[u][v]); }
        auto r=general_weighted_matching(g); CHECK(r.weight==dp.back()); long long total=0; std::set<int> vertices;
        for(int id:r.edge_ids) { auto e=g.edges()[id]; CHECK(vertices.insert(e.from).second && vertices.insert(e.to).second); CHECK(r.mate[e.from]==e.to && r.mate[e.to]==e.from); total+=e.cost; } CHECK(r.weight==total);
    }
    Graph<long long> extreme(4); extreme.add_edge(0,1,LLONG_MAX); extreme.add_edge(2,3,LLONG_MAX); extreme.add_edge(1,2,LLONG_MAX); CHECK(general_weighted_matching(extreme).weight==OptimizationWide(LLONG_MAX)*2);
    for(int trial=0;trial<500;++trial) {
        int l=int(rng()%6),r=int(rng()%6); std::vector<RankedBipartiteEdge> edges;
        for(int a=0;a<l;++a) for(int b=0;b<r;++b) if(rng()%2) edges.push_back({a,b,1+int(rng()%4)});
        auto result=rank_maximal_matching(l,r,edges); std::vector<int> best(4),profile(4);
        std::function<void(int,int)> dfs=[&](int a,int used) {
            if(a==l) { best=std::max(best,profile); return; }
            dfs(a+1,used); for(auto e:edges) if(e.left==a && !(used>>e.right&1)) { ++profile[e.rank-1]; dfs(a+1,used|(1<<e.right)); --profile[e.rank-1]; }
        }; dfs(0,0); std::vector<int> actual(4); for(int i=0;i<int(result.ranks.size());++i) actual[result.ranks[i]-1]=result.profile[i]; CHECK(actual==best);
    }
    for(int trial=0;trial<100;++trial) {
        int n=2+int(rng()%7); DynamicUnitFlow flow(n,0,n-1);
        for(int step=0;step<180;++step) {
            if(flow.edge_count()==0 || rng()%3==0) flow.add_edge(int(rng()%n),int(rng()%n),rng()%2);
            else flow.set_enabled(int(rng()%flow.edge_count()),rng()%2);
            int best=100000; for(int mask=0;mask<(1<<n);++mask) if((mask&1) && !(mask>>(n-1)&1)) {
                int cut=0; for(int id=0;id<flow.edge_count();++id) { auto e=flow.get_edge(id); if((mask>>e.from&1) && !(mask>>e.to&1)) cut+=e.capacity; } best=std::min(best,cut);
            }
            CHECK(flow.value()==best); auto side=flow.min_cut(); int cut=0; std::vector<int> net(n);
            for(int id=0;id<flow.edge_count();++id) { auto e=flow.get_edge(id); CHECK(0<=e.flow && e.flow<=e.capacity); net[e.from]+=e.flow; net[e.to]-=e.flow; if(side[e.from] && !side[e.to]) cut+=e.capacity; }
            CHECK(cut==best); CHECK(net[0]==best && net[n-1]==-best); for(int v=1;v<n-1;++v) CHECK(net[v]==0);
        }
    }
    Graph<int> dense(151); for(int u=0;u<151;++u) for(int v=u+1;v<151;++v) dense.add_edge(u,v,7);
    CHECK(general_weighted_matching(dense).weight==75*7);
    std::vector<RankedBipartiteEdge> many_ranks;
    for(int i=0;i<80;++i) many_ranks.push_back({i,i,1+i*1000000});
    CHECK(rank_maximal_matching(80,80,many_ranks).edge_ids.size()==80);
    CHECK(CostScalingBFlow<>(0).solve()->cost==0);
    CostScalingBFlow<> bounds(1); bounds.add_edge(0,0,LLONG_MIN,LLONG_MAX,LLONG_MIN); CHECK(bounds.solve()->flow[0]==LLONG_MAX);
    throws<std::invalid_argument>([]{CostScalingBFlow<> f(-1);});
    throws<std::invalid_argument>([]{DynamicUnitFlow f(2,0,0);});
    throws<std::invalid_argument>([]{rank_maximal_matching(1,1,{{0,0,0}});});
    std::cout<<"flow: exhaustive b-flows, 40 capacity-scale comparisons, 2500 weighted matchings, 500 ranked matchings, 18000 dynamic updates passed\n";
}
