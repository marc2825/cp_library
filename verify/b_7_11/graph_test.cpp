#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <vector>
#include "graph_basic/all.hpp"
#include "graph_connectivity/all.hpp"
#include "graph_flow/dinic.hpp"
#define CHECK(x) do{if(!(x)){std::cerr<<"check failed line "<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
using G=cp::Graph<long long>;
std::mt19937 rng(7112026);
int draw(int n){return static_cast<int>(rng()%n);}
std::vector<bool> reach(const G& g,int s,int removed=-1){std::vector<bool> seen(g.size());if(s==removed)return seen;seen[s]=true;std::vector<int> q{s};for(std::size_t j=0;j<q.size();++j)for(auto e:g[q[j]])if(e.to!=removed && !seen[e.to]){seen[e.to]=true;q.push_back(e.to);}return seen;}
int main(){
 for(int trial=0;trial<600;++trial){int n=1+draw(7),m=draw(12);G g(n);for(int i=0;i<m;++i)g.add_edge(draw(n),draw(n),draw(13)-6);
  std::vector<std::pair<long long,unsigned>> trees;for(unsigned mask=0;mask<(1u<<m);++mask)if(__builtin_popcount(mask)==n-1){cp::DisjointSetUnion dsu(n);long long cost=0;bool ok=true;for(auto e:g.edges())if(mask>>e.idx&1){if(!dsu.merge(e.from,e.to))ok=false;cost+=e.cost;}if(ok && dsu.component_count()==1)trees.emplace_back(cost,mask);}
  std::sort(trees.begin(),trees.end());auto s=cp::mst_sensitivity(g);CHECK(bool(s)==!trees.empty());if(s){CHECK(s->cost==trees[0].first);unsigned chosen=0;for(int i=0;i<m;++i)if(s->in_tree[i])chosen|=1u<<i;std::optional<long long> different,strict;
   for(auto [cost,mask]:trees){if(mask!=chosen && !different)different=cost;if(cost>s->cost && !strict)strict=cost;}
   CHECK(bool(s->different)==bool(different));CHECK(bool(s->strictly_larger)==bool(strict));if(different)CHECK(s->different->cost==*different);if(strict)CHECK(s->strictly_larger->cost==*strict);
   for(int id=0;id<m;++id){std::optional<long long> forced,forbidden;for(auto [cost,mask]:trees){if(mask>>id&1){if(!forced)forced=cost;}else if(!forbidden)forbidden=cost;}CHECK(s->forced_cost[id]==forced);CHECK(s->forbidden_cost[id]==forbidden);}
   for(bool strict_mode:{false,true}){auto next=cp::second_best_spanning_tree(g,strict_mode);auto expect=strict_mode?strict:different;CHECK(bool(next)==bool(expect));if(next){CHECK(next->cost==*expect);CHECK(next->edges.size()==static_cast<std::size_t>(n-1));}}
  }
  std::vector<std::pair<long long,long long>> points(n);G complete(n);for(auto& p:points)p={draw(31)-15,draw(31)-15};for(int u=0;u<n;++u)for(int v=u+1;v<n;++v)complete.add_edge(u,v,std::abs(points[u].first-points[v].first)+std::abs(points[u].second-points[v].second));CHECK(cp::manhattan_mst(points).cost==cp::kruskal(complete).cost);
 }
 for(int trial=0;trial<450;++trial){int n=1+draw(6),m=draw(11),root=draw(n);G g(n,cp::GraphKind::Directed);for(int i=0;i<m;++i)g.add_edge(draw(n),draw(n),draw(17)-8);std::optional<long long> optimum;
  for(unsigned mask=0;mask<(1u<<m);++mask)if(__builtin_popcount(mask)==n-1){G sub(n,cp::GraphKind::Directed);std::vector<int> deg(n);long long cost=0;for(auto e:g.edges())if(mask>>e.idx&1){++deg[e.to];sub.add_edge(e.from,e.to);cost+=e.cost;}bool ok=deg[root]==0;for(int v=0;v<n;++v)if(v!=root && deg[v]!=1)ok=false;auto seen=reach(sub,root);if(ok && std::all_of(seen.begin(),seen.end(),[](bool x){return x;}))if(!optimum || cost<*optimum)optimum=cost;}
  auto answer=cp::directed_mst(g,root);CHECK(bool(answer)==bool(optimum));if(answer){CHECK(answer->cost==*optimum);G sub(n,cp::GraphKind::Directed);long long cost=0;for(int v=0;v<n;++v)if(v!=root){int id=answer->parent_edge[v];CHECK(id>=0 && id<m);auto e=g.edges()[id];CHECK(e.to==v && e.from==answer->parent[v]);cost+=e.cost;sub.add_edge(e.from,e.to);}CHECK(cost==*optimum);auto seen=reach(sub,root);CHECK(std::count(seen.begin(),seen.end(),true)==n);}
 }
 for(bool directed:{false,true})for(int trial=0;trial<350;++trial){int n=1+draw(7),m=draw(12);G g(n,directed?cp::GraphKind::Directed:cp::GraphKind::Undirected);for(int i=0;i<m;++i)g.add_edge(draw(n),draw(n),draw(8));std::optional<long long> best;std::vector<bool> used(n),edge_used(m);
  std::function<void(int,int,long long)> dfs=[&](int s,int v,long long cost){for(auto e:g[v])if(!edge_used[e.idx]){long long next=cost+e.cost;if(e.to==s){if(!best || next<*best)best=next;}else if(!used[e.to]){used[e.to]=true;edge_used[e.idx]=true;dfs(s,e.to,next);edge_used[e.idx]=false;used[e.to]=false;}}};
  for(int v=0;v<n;++v){used[v]=true;dfs(v,v,0);used[v]=false;}
  auto cycle=directed?cp::shortest_directed_cycle(g):cp::shortest_undirected_cycle(g);CHECK(bool(cycle)==bool(best));if(cycle){CHECK(cycle->cost==*best);CHECK(cycle->vertices.front()==cycle->vertices.back());CHECK(cycle->vertices.size()==cycle->edge_ids.size()+1);std::set<int> ids;long long cost=0;for(std::size_t i=0;i<cycle->edge_ids.size();++i){int id=cycle->edge_ids[i];CHECK(ids.insert(id).second);auto e=g.edges()[id];int u=cycle->vertices[i],v=cycle->vertices[i+1];CHECK((e.from==u && e.to==v)||(!directed && e.from==v && e.to==u));cost+=e.cost;}CHECK(cost==*best);}
 }
 for(int trial=0;trial<600;++trial){int n=1+draw(9);G g(n,cp::GraphKind::Directed);int root=draw(n);for(int i=0,m=draw(30);i<m;++i)g.add_edge(draw(n),draw(n));auto d=cp::dominator_tree(g,root);auto initial=reach(g,root);for(int u=0;u<n;++u){auto without=reach(g,root,u);for(int v=0;v<n;++v)CHECK(d.dominates(u,v)==(initial[v] && !without[v]));}for(int v=0;v<n;++v){if(!initial[v])CHECK(d.idom[v]==-1);else if(v==root)CHECK(d.idom[v]==v);else{int p=d.idom[v];CHECK(p!=v && d.dominates(p,v));for(int u=0;u<n;++u)if(u!=v && d.dominates(u,v))CHECK(d.dominates(u,p));}}
  cp::IncrementalSCC incremental(n);G added(n,cp::GraphKind::Directed);for(int i=0;i<35;++i){int u=draw(n),v=draw(n);incremental.add_edge(u,v);added.add_edge(u,v);auto expected=cp::scc(added),actual=incremental.components();CHECK(incremental.component_count()==expected.size());for(int a=0;a<n;++a)for(int b=0;b<n;++b){CHECK(incremental.same(a,b)==expected.same(a,b));CHECK(actual.same(a,b)==expected.same(a,b));}for(auto e:added.edges())CHECK(actual.component[e.from]<=actual.component[e.to]);}
 }
 for(int trial=0;trial<600;++trial){int n=1+draw(10);G g(n);for(int i=0,m=draw(25);i<m;++i)g.add_edge(draw(n),draw(n));auto c=cp::three_edge_connected_components(g);for(int s=0;s<n;++s)for(int t=s+1;t<n;++t){cp::Dinic<int> f(n);for(auto e:g.edges())if(e.from!=e.to){f.add_edge(e.from,e.to,1);f.add_edge(e.to,e.from,1);}CHECK((c.component[s]==c.component[t])==(f.max_flow(s,t,3)>=3));}
 }
 for(int trial=0;trial<350;++trial){int n=1+draw(7),m=draw(10);G g(n);for(int i=0;i<m;++i)g.add_edge(draw(n),draw(n));std::vector<int> cycle_count(m);for(unsigned mask=1;mask<(1u<<m);++mask){cp::DisjointSetUnion dsu(n);std::vector<int> deg(n);int start=-1;for(auto e:g.edges())if(mask>>e.idx&1){deg[e.from]++;deg[e.to]++;dsu.merge(e.from,e.to);start=e.from;}bool ok=true;for(int v=0;v<n;++v)if(deg[v] && (deg[v]!=2 || !dsu.same(v,start)))ok=false;if(ok)for(int id=0;id<m;++id)if(mask>>id&1)++cycle_count[id];}bool expected=std::all_of(cycle_count.begin(),cycle_count.end(),[](int x){return x<=1;});auto c=cp::cactus_decomposition(g);CHECK(c.is_cactus==expected);if(expected){std::vector<int> used(m);for(const auto& block:c.blocks)for(int id:block.edge_ids)++used[id];for(int x:used)CHECK(x==1);}
 }
 for(int trial=0;trial<450;++trial){int n=draw(8);G g(n);std::vector<std::vector<bool>> adj(n,std::vector<bool>(n));for(int u=0;u<n;++u)for(int v=u+1;v<n;++v)if(draw(2)){g.add_edge(u,v);adj[u][v]=adj[v][u]=true;}bool chordal=true;for(unsigned mask=1;mask<(1u<<n);++mask)if(__builtin_popcount(mask)>=4){G sub(n);bool cycle=true;for(int v=0;v<n;++v)if(mask>>v&1){int degree=0;for(int u=0;u<n;++u)if((mask>>u&1) && adj[u][v]){++degree;sub.add_edge(u,v);}if(degree!=2)cycle=false;}if(cycle){int start=__builtin_ctz(mask);auto seen=reach(sub,start);if(std::count(seen.begin(),seen.end(),true)==__builtin_popcount(mask))chordal=false;}}auto c=cp::chordal_graph(g);CHECK(c.is_chordal==chordal);if(chordal){int clique=0;for(unsigned mask=0;mask<(1u<<n);++mask){bool ok=true;for(int u=0;u<n;++u)if(mask>>u&1)for(int v=u+1;v<n;++v)if((mask>>v&1) && !adj[u][v])ok=false;if(ok)clique=std::max(clique,__builtin_popcount(mask));}CHECK(c.chromatic_number==clique);for(auto e:g.edges())CHECK(c.color[e.from]!=c.color[e.to]);}
  std::function<bool(std::vector<std::vector<bool>>,unsigned)> feasible=[&](auto a,unsigned mask){if(!mask)return true;for(int v=0;v<n;++v)if(mask>>v&1){std::vector<int> neighbors;for(int u=0;u<n;++u)if((mask>>u&1) && a[v][u])neighbors.push_back(u);if(neighbors.size()<=2){auto b=a;if(neighbors.size()==2)b[neighbors[0]][neighbors[1]]=b[neighbors[1]][neighbors[0]]=true;if(feasible(b,mask^(1u<<v)))return true;}}return false;};auto td=cp::tree_decomposition_width_2(g);CHECK(bool(td)==feasible(adj,(1u<<n)-1));if(td){for(auto bag:td->bags)CHECK(bag.size()<=3);for(auto e:g.edges()){bool covered=false;for(auto bag:td->bags)if(std::find(bag.begin(),bag.end(),e.from)!=bag.end() && std::find(bag.begin(),bag.end(),e.to)!=bag.end())covered=true;CHECK(covered);}for(int v=0;v<n;++v){std::vector<bool> contains(n);int start=-1,total=0;for(int i=0;i<n;++i)if(std::find(td->bags[i].begin(),td->bags[i].end(),v)!=td->bags[i].end()){contains[i]=true;start=i;++total;}std::vector<int> q{start};contains[start]=false;for(std::size_t j=0;j<q.size();++j)for(auto e:td->forest[q[j]])if(contains[e.to]){contains[e.to]=false;q.push_back(e.to);}CHECK(static_cast<int>(q.size())==total);}}
 }
 for(int trial=0;trial<300;++trial){int n=draw(35);G g(n,cp::GraphKind::Directed);for(int u=0;u<n;++u)for(int v=u+1;v<n;++v)if(draw(5)==0)g.add_edge(u,v);cp::DAGReachability index(g);std::vector<std::pair<int,int>> queries;for(int i=0;i<180 && n;++i)queries.emplace_back(draw(n),draw(n));auto batch=cp::dag_reachability(g,queries);for(std::size_t i=0;i<queries.size();++i){auto [u,v]=queries[i];bool expected=reach(g,u)[v];CHECK(index.reachable(u,v)==expected);CHECK(batch[i]==expected);}}
 {G chain(100000,cp::GraphKind::Directed);for(int i=1;i<chain.size();++i)chain.add_edge(i-1,i);auto d=cp::dominator_tree(chain);CHECK(d.idom.back()==99998);}
 {G empty(0);CHECK(cp::directed_mst(G(0,cp::GraphKind::Directed))->cost==0);CHECK(cp::mst_sensitivity(empty)->cost==0);CHECK(cp::cactus_decomposition(empty).is_cactus);CHECK(cp::tree_decomposition_width_2(empty));}
 {G k4(4);for(int u=0;u<4;++u)for(int v=u+1;v<4;++v)k4.add_edge(u,v);CHECK(!cp::tree_decomposition_width_2(k4));bool rejected=false;try{cp::directed_mst(k4);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);G cycle(2,cp::GraphKind::Directed);cycle.add_edge(0,1);cycle.add_edge(1,0);rejected=false;try{cp::DAGReachability invalid(cycle);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);}
 std::cout<<"graph independent/exhaustive/random checks passed\n";
}
