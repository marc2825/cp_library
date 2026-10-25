#include "common.hpp"
#include "graph_combinatorics/connected_graph_count.hpp"
#include "graph_combinatorics/strongly_connected_graph_count.hpp"
#include "graph_combinatorics/euler_circuit_count.hpp"
#include "basic_utility/static_modint.hpp"
using namespace cp;
int main() {
    auto connected=connected_graph_counts(12),strong=strongly_connected_graph_counts(10);
    CHECK(connected[0]==BigInt(0) && connected[1]==BigInt(1) && connected[6]==BigInt(26704));
    CHECK(strong[0]==BigInt(0) && strong[1]==BigInt(1) && strong[5]==BigInt(565080));
    for(int directed=0;directed<2;++directed)for(int n=1;n<=(directed?4:6);++n){
        std::vector<std::pair<int,int>> edges;for(int u=0;u<n;++u)for(int v=0;v<n;++v)if(directed?u!=v:u<v)edges.emplace_back(u,v);
        int count=0;for(int mask=0;mask<(1<<int(edges.size()));++mask){std::vector<int> reach(n);for(int v=0;v<n;++v)reach[v]=1<<v;for(int j=0;j<int(edges.size());++j)if(mask>>j&1){auto [u,v]=edges[j];reach[u]|=1<<v;if(!directed)reach[v]|=1<<u;}for(int k=0;k<n;++k)for(int v=0;v<n;++v)if(reach[v]>>k&1)reach[v]|=reach[k];bool good=true;for(int x:reach)good=good && x==(1<<n)-1;count+=good;}
        CHECK(BigInt(count)==(directed?strong[n]:connected[n]));
    }
    auto cm=connected_graph_counts<StaticModInt<12>>(12),sm=strongly_connected_graph_counts<StaticModInt<12>>(10);
    for(int i=0;i<=12;++i)CHECK(cm[i].val()==(connected[i]%BigInt(12)).to_integral<unsigned>());
    for(int i=0;i<=10;++i)CHECK(sm[i].val()==(strong[i]%BigInt(12)).to_integral<unsigned>());
    std::mt19937 rng(491);
    for(int trial=0;trial<800;++trial){
        int n=1+int(rng()%5),m=int(rng()%9);Graph<int> g(n,GraphKind::Directed);
        for(int i=0;i<m;++i)g.add_edge(int(rng()%n),int(rng()%n));
        int root=int(rng()%n),total=0;std::vector<int> first(m);
        std::function<void(int,int,int)> dfs=[&](int v,int mask,int head){if(mask==(1<<m)-1){if(v==root){++total;if(head>=0)++first[head];}return;}for(auto e:g[v])if(!(mask>>e.idx&1))dfs(e.to,mask|(1<<e.idx),head<0?e.idx:head);};dfs(root,0,-1);
        CHECK(count_euler_circuits(g,root)==total);CHECK(count_euler_circuits<2>(g,root)==total%2);
        for(auto e:g[root])CHECK(count_euler_circuits(g,root,e.idx)==first[e.idx]);
    }
    Graph<int> loops(3,GraphKind::Directed);for(int i=0;i<8;++i)loops.add_edge(1,1);CHECK(count_euler_circuits(loops,1)==40320);CHECK(count_euler_circuits(loops,1,0)==5040);CHECK(count_euler_circuits(loops,0)==0);
    throws<std::invalid_argument>([]{connected_graph_counts(-1);});throws<std::out_of_range>([]{count_euler_circuits(Graph<int>(0,GraphKind::Directed),0);});
    std::cout<<"counts: all undirected graphs n<=6, directed graphs n<=4, composite-mod recurrence, 800 Euler edge-sequence enumerations passed\n";
}
