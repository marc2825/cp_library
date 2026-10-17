#include <algorithm>
#include <climits>
#include <cstdint>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>
#include <vector>
#include "graph_basic/all.hpp"
void check(bool ok){if(!ok)throw std::runtime_error("shortest-path mismatch");}
template<class T> void restore(const cp::Graph<T>& g,const cp::ShortestPathResult<T>& p){
    for(int v=0;v<g.size();++v){auto vs=p.vertex_path(v),es=p.edge_path(v);if(!p.reachable(v)){check(vs.empty()&&es.empty());continue;}
        check(vs.front()==p.source&&vs.back()==v&&vs.size()==es.size()+1);T cost=0;
        for(int i=0;i<int(es.size());++i){auto e=g.edges()[es[i]];check((e.from==vs[i]&&e.to==vs[i+1])||(!g.directed()&&e.to==vs[i]&&e.from==vs[i+1]));cost+=e.cost;}
        check(cost==p.dist[v]);
    }
}
int main(){
    std::mt19937 rng(2381723);
    for(int trial=0;trial<700;++trial){
        int n=1+rng()%20;bool directed=rng()%2;cp::Graph<long long> g(n,directed?cp::GraphKind::Directed:cp::GraphKind::Undirected);
        std::vector<std::vector<long long>> d(n,std::vector<long long>(n,1000000));for(int i=0;i<n;++i)d[i][i]=0;
        for(int e=0;e<50;++e){int u=rng()%n,v=rng()%n;long long w=rng()%7;g.add_edge(u,v,w);d[u][v]=std::min(d[u][v],w);if(!directed)d[v][u]=std::min(d[v][u],w);}
        for(int k=0;k<n;++k)for(int u=0;u<n;++u)for(int v=0;v<n;++v)d[u][v]=std::min(d[u][v],d[u][k]+d[k][v]);
        for(int s=0;s<n;++s){auto a=cp::radix_heap_dijkstra(g,s),b=cp::dial_shortest_path(g,s,6);for(int v=0;v<n;++v)check(a.dist[v]==b.dist[v]&&(d[s][v]==1000000?!a.reachable(v):a.dist[v]==d[s][v]));restore(g,a);restore(g,b);}
        if(!directed){auto c=cp::complement_bfs(g,0);std::vector<std::vector<bool>>adj(n,std::vector<bool>(n));for(auto e:g.edges())adj[e.from][e.to]=adj[e.to][e.from]=true;
            std::vector<int> dist(n,-1),q{0};dist[0]=0;for(std::size_t i=0;i<q.size();++i)for(int v=0;v<n;++v)if(v!=q[i]&&!adj[q[i]][v]&&dist[v]<0){dist[v]=dist[q[i]]+1;q.push_back(v);}
            for(int v=0;v<n;++v){check(c.reachable(v)==(dist[v]>=0));if(dist[v]>=0){check(c.dist[v]==dist[v]);auto p=c.vertex_path(v);check(p.front()==0&&p.back()==v&&int(p.size())==dist[v]+1);for(int j=1;j<int(p.size());++j)check(!adj[p[j-1]][p[j]]);}}
        }
    }
    for(int trial=0;trial<500;++trial){
        int n=1+rng()%15;cp::Graph<long long> g(n,cp::GraphKind::Directed);std::vector<int> potential(n);for(int& h:potential)h=int(rng()%21)-10;
        for(int i=0;i<35;++i){int u=rng()%n,v=rng()%n;g.add_edge(u,v,static_cast<long long>(rng()%8)+potential[v]-potential[u]);}
        auto all=cp::johnson(g);check(bool(all));for(int s=0;s<n;++s){std::vector<long long>d(n,1000000);d[s]=0;for(int step=1;step<n;++step)for(auto e:g.edges())if(d[e.from]!=1000000)d[e.to]=std::min(d[e.to],d[e.from]+e.cost);for(int v=0;v<n;++v)check(d[v]==1000000?!(*all)[s].reachable(v):d[v]==(*all)[s].dist[v]);restore(g,(*all)[s]);}
        cp::Graph<long long> dag(n,cp::GraphKind::Directed);for(int i=0;i<35;++i){int u=rng()%n,v=rng()%n;if(u<v)dag.add_edge(u,v,int(rng()%21)-10);}
        auto row=cp::dag_shortest_path(dag,0);check(bool(row));std::vector<long long>d(n,1000000);d[0]=0;for(int i=1;i<n;++i)for(auto e:dag.edges())if(d[e.from]!=1000000)d[e.to]=std::min(d[e.to],d[e.from]+e.cost);for(int v=0;v<n;++v)check(d[v]==1000000?!row->reachable(v):d[v]==row->dist[v]);restore(dag,*row);
    }
    cp::Graph<long long> cycle(3,cp::GraphKind::Directed);cycle.add_edge(1,2,-1);cycle.add_edge(2,1,0);check(!cp::johnson(cycle));check(!cp::dag_shortest_path(cycle,0));
    cp::Graph<long long> empty(0,cp::GraphKind::Directed);check(cp::johnson(empty)->empty());
    cp::Graph<std::uint64_t> extreme(2,cp::GraphKind::Directed);extreme.add_edge(0,1,UINT64_MAX-1);check(cp::radix_heap_dijkstra(extreme).dist[1]==UINT64_MAX-1);
    cp::Graph<long long> overflow(2,cp::GraphKind::Directed);overflow.add_edge(0,1,LLONG_MAX);bool thrown=false;try{cp::radix_heap_dijkstra(overflow);}catch(const std::overflow_error&){thrown=true;}check(thrown);
    cp::Graph<int> zero(4);zero.add_edge(0,1,0);zero.add_edge(1,2,0);check(cp::dial_shortest_path(zero,0,0).dist[2]==0);
    cp::Graph<long long> large(200000);for(int i=1;i<large.size();++i)large.add_edge(i-1,i,1);check(cp::dial_shortest_path(large,0,1).dist.back()==199999);check(cp::radix_heap_dijkstra(large).dist.back()==199999);check(cp::complement_bfs(large).dist.back()==1);
    std::cout<<"shortest: 700 nonnegative graphs, 500 potential/DAG graphs, all restored paths, complement BFS and 200000 vertices passed\n";
}
