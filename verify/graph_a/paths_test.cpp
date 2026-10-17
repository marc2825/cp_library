#include <algorithm>
#include <iostream>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>
#include "graph_basic/k_shortest_paths.hpp"
#include "graph_basic/k_shortest_walks.hpp"
#define check(condition) do { if(!(condition)) throw std::runtime_error(std::string("k-path/walk mismatch at line ")+std::to_string(__LINE__)); } while(false)
void validate(const cp::Graph<long long>& g,int s,int t,const cp::WeightedPath<long long>& p,bool simple){
    check(p.vertices.front()==s&&p.vertices.back()==t&&p.vertices.size()==p.edge_ids.size()+1);long long cost=0;
    for(int i=0;i<int(p.edge_ids.size());++i){auto e=g.edges()[p.edge_ids[i]];
check((e.from==p.vertices[i]&&e.to==p.vertices[i+1])||(!g.directed()&&e.to==p.vertices[i]&&e.from==p.vertices[i+1]));cost+=e.cost;}
    check(cost==p.cost);if(simple)check(std::set<int>(p.vertices.begin(),p.vertices.end()).size()==p.vertices.size());
}
int main(){
    std::mt19937 rng(871237);
    for(int trial=0;trial<650;++trial){
        int n=1+rng()%7,s=rng()%n,t=rng()%n;cp::Graph<long long>g(n,rng()%2?cp::GraphKind::Directed:cp::GraphKind::Undirected);
        for(int i=0;i<12;++i)g.add_edge(rng()%n,rng()%n,rng()%5);
        std::vector<long long>costs;std::vector<bool>visited(n);std::set<std::vector<int>>all_paths;
        std::vector<int>path;
        auto dfs=[&](auto&& self,int v,long long d)->void{if(v==t){costs.push_back(d);all_paths.insert(path);return;}visited[v]=true;for(auto e:g[v])if(!visited[e.to]){path.push_back(e.idx);self(self,e.to,d+e.cost);path.pop_back();}visited[v]=false;};dfs(dfs,s,0);std::sort(costs.begin(),costs.end());
        int k=1+rng()%25;auto got=cp::k_shortest_paths(g,s,t,k);
check(got.size()==std::min(costs.size(),std::size_t(k)));std::set<std::vector<int>>unique;
        for(int i=0;i<int(got.size());++i){check(got[i].cost==costs[i]);validate(g,s,t,got[i],true);
check(all_paths.count(got[i].edge_ids)&&unique.insert(got[i].edge_ids).second);}
        std::vector<std::vector<std::pair<int,long long>>> arcs(n);
        for(auto e:g.edges()){arcs[e.from].emplace_back(e.to,e.cost);if(!g.directed()&&e.from!=e.to)arcs[e.to].emplace_back(e.from,e.cost);}
        using Entry=std::pair<long long,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>>queue;queue.emplace(0,s);std::vector<int>popped(n);std::vector<long long>expected;
        while(!queue.empty()&&int(expected.size())<k){auto[d,v]=queue.top();queue.pop();if(popped[v]++>=k)continue;if(v==t)expected.push_back(d);for(auto [to,cost]:arcs[v])if(popped[to]<k)queue.emplace(d+cost,to);}
        auto walks=cp::k_shortest_walks(g,s,t,k);
check(walks.costs==expected);unique.clear();
        for(int i=0;i<int(walks.costs.size());++i){auto p=walks.path(i);validate(g,s,t,p,false);
check(unique.insert(p.edge_ids).second);}
        check(cp::k_shortest_paths(g,s,t,0).empty()&&cp::k_shortest_walks(g,s,t,0).costs.empty());
    }
    cp::Graph<long long>g(200000,cp::GraphKind::Directed);for(int i=1;i<g.size();++i)g.add_edge(i-1,i,1);g.add_edge(g.size()-1,g.size()-1,0);
    auto result=cp::k_shortest_walks(g,0,g.size()-1,2000);
check(result.costs.size()==2000&&result.costs.back()==199999);auto path=result.path(1999);
check(path.edge_ids.size()==201998);
    std::cout<<"paths: 650 multigraphs against all simple paths and uniform-cost walks, zero cycles, reconstruction and 200000 vertices passed\n";
}
