#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <tuple>
#include <vector>
#include "graph_flow/all.hpp"
#include "combinatorial_optimization/detail/arithmetic.hpp"
#define CHECK(x) do{if(!(x)){std::cerr<<"check failed line "<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
std::mt19937 rng(39112026);int draw(int n){return int(rng()%n);}
int main(){
 for(int trial=0;trial<1000;++trial){int n=1+draw(5),m=draw(8);cp::MinCostBFlow<> problem(n);cp::MinCostCirculation<> circulation(n);std::vector<std::tuple<int,int,int,int,int>> edges;std::vector<int> b(n);for(int i=0;i<n;++i)b[i]=draw(5)-2;for(int i=0;i<m;++i){int u=draw(n),v=draw(n),l=draw(3)-1,r=l+draw(3),c=draw(9)-4;edges.emplace_back(u,v,l,r,c);problem.add_edge(u,v,l,r,c);circulation.add_edge(u,v,l,r,c);}for(int v=0;v<n;++v)problem.set_balance(v,b[v]);std::optional<long long> best,best_cycle;std::vector<int> net(n);
  std::function<void(int,long long)> enumerate=[&](int i,long long cost){if(i==m){if(net==b && (!best || cost<*best))best=cost;if(std::all_of(net.begin(),net.end(),[](int x){return x==0;}) && (!best_cycle || cost<*best_cycle))best_cycle=cost;return;}auto [u,v,l,r,c]=edges[i];for(int f=l;f<=r;++f){net[u]+=f;net[v]-=f;enumerate(i+1,cost+f*c);net[u]-=f;net[v]+=f;}};enumerate(0,0);
  auto result=problem.solve(),cycle=circulation.solve();CHECK(bool(result)==bool(best));CHECK(bool(cycle)==bool(best_cycle));if(cycle)CHECK(cycle->cost==*best_cycle);if(result){CHECK(result->cost==*best);std::fill(net.begin(),net.end(),0);long long cost=0;for(int i=0;i<m;++i){auto [u,v,l,r,c]=edges[i];long long f=result->flow[i];CHECK(l<=f && f<=r);net[u]+=int(f);net[v]-=int(f);cost+=f*c;if(f<r)CHECK(c+result->potential[u]-result->potential[v]>=0);if(f>l)CHECK(-c+result->potential[v]-result->potential[u]>=0);}CHECK(cost==*best);CHECK(net==b);}
 }
 {using L=long long;auto max=std::numeric_limits<L>::max();cp::MinCostCirculation<> f(1);f.add_edge(0,0,0,max,-max);auto r=f.solve();CHECK(r && r->cost==-cp::OptimizationWide(max)*max && r->flow[0]==max);cp::MinCostBFlow<> negative(2);negative.add_edge(0,1,-3,-1,5);negative.set_balance(0,-2);negative.set_balance(1,2);CHECK(negative.solve()->cost==-10);}
 for(int trial=0;trial<800;++trial){int n=2+draw(8),m=draw(35);cp::PushRelabel<long long> p(n);cp::Dinic<long long> dinic(n);cp::Graph<long long> undirected(n);std::vector<std::tuple<int,int,int>> edges;for(int i=0;i<m;++i){int u=draw(n),v=draw(n),c=draw(9);p.add_edge(u,v,c);dinic.add_edge(u,v,c);undirected.add_edge(u,v,c);edges.emplace_back(u,v,c);}auto r=p.solve(0,n-1);CHECK(r.value==dinic.max_flow(0,n-1));CHECK(p.max_flow(0,n-1)==r.value);std::vector<long long> net(n);long long cut=0;for(int i=0;i<m;++i){auto [u,v,c]=edges[i];CHECK(r.flow[i]>=0 && r.flow[i]<=c);net[u]+=r.flow[i];net[v]-=r.flow[i];if(r.source_side[u] && !r.source_side[v])cut+=c;}CHECK(cut==r.value);CHECK(net[0]==r.value && net[n-1]==-r.value);for(int v=1;v<n-1;++v)CHECK(net[v]==0);
  long long optimum=std::numeric_limits<long long>::max();for(unsigned mask=1;mask+1<(1u<<n);++mask){long long c=0;for(auto [u,v,w]:edges)if(((mask>>u)^(mask>>v))&1)c+=w;optimum=std::min(optimum,c);}auto global=cp::stoer_wagner(undirected);CHECK(global && global->cost==optimum);long long actual=0;for(auto [u,v,w]:edges)if(global->side[u]!=global->side[v])actual+=w;CHECK(actual==optimum);CHECK(std::count(global->side.begin(),global->side.end(),true)>0 && std::count(global->side.begin(),global->side.end(),true)<n);
 }
 {cp::PushRelabel<unsigned long long> f(3);auto big=std::numeric_limits<unsigned long long>::max();f.add_edge(0,1,big);f.add_edge(0,1,big);f.add_edge(1,2,7);CHECK(f.solve(0,2).value==7);}
 for(int trial=0;trial<1200;++trial){int l=draw(5),r=draw(5),m=l && r?draw(10):0;std::vector<std::pair<int,int>> edges;std::vector<std::tuple<int,int,long long>> weighted;for(int i=0;i<m;++i){int u=draw(l),v=draw(r);edges.emplace_back(u,v);weighted.emplace_back(u,v,draw(15)-7);}int maximum=0;long long max_weight=0;std::vector<bool> allowed(m);std::vector<unsigned> matchings;
  for(unsigned mask=0;mask<(1u<<m);++mask){unsigned left=0,right=0;bool ok=true;long long weight=0;for(int i=0;i<m;++i)if(mask>>i&1){auto [u,v]=edges[i];if((left>>u&1)||(right>>v&1))ok=false;left|=1u<<u;right|=1u<<v;weight+=std::get<2>(weighted[i]);}if(ok){int k=__builtin_popcount(mask);if(k>maximum){maximum=k;matchings.clear();}if(k==maximum)matchings.push_back(mask);max_weight=std::max(max_weight,weight);}}
  for(auto mask:matchings)for(int i=0;i<m;++i)if(mask>>i&1)allowed[i]=true;
  auto dm=cp::dulmage_mendelsohn(l,r,edges);CHECK(dm.matching.cardinality==maximum);CHECK(dm.allowed_edge==allowed);for(auto [u,v]:edges)CHECK(dm.component[u]<=dm.component[l+v]);auto mw=cp::maximum_weight_bipartite_matching(l,r,weighted);CHECK(mw.weight==max_weight);std::set<int> lu,rv;long long weight=0;for(auto [u,v]:mw.pairs){CHECK(lu.insert(u).second && rv.insert(v).second);long long w=std::numeric_limits<long long>::lowest();for(auto [a,b,c]:weighted)if(a==u && b==v)w=std::max(w,c);CHECK(w!=std::numeric_limits<long long>::lowest());weight+=w;}CHECK(weight==mw.weight);
  auto colored=cp::bipartite_edge_coloring(l,r,edges);int delta=0;std::vector<int> degree(l+r);for(auto [u,v]:edges){delta=std::max(delta,++degree[u]);delta=std::max(delta,++degree[l+v]);}CHECK(colored.color_count==delta);CHECK(colored.color.size()==edges.size());for(int i=0;i<m;++i){CHECK(0<=colored.color[i] && colored.color[i]<delta);for(int j=0;j<i;++j)if(edges[i].first==edges[j].first || edges[i].second==edges[j].second)CHECK(colored.color[i]!=colored.color[j]);}
 }
 {cp::MinCostCirculation<> f(1);f.add_edge(0,0,0,1,std::numeric_limits<long long>::min());CHECK(f.solve()->cost==cp::OptimizationWide(std::numeric_limits<long long>::min()));bool rejected=false;try{f.add_edge(0,0,2,1,0);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);CHECK(cp::maximum_weight_bipartite_matching(1,1,{{0,0,-8}}).weight==0);rejected=false;try{cp::maximum_weight_bipartite_matching(1,1,{{0,1,8}});}catch(const std::out_of_range&){rejected=true;}CHECK(rejected);}
 std::cout<<"flow bounded enumeration/cut/matching checks passed\n";
}
