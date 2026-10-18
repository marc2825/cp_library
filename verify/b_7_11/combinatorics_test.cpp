#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <vector>
#include "graph_combinatorics/all.hpp"
#include "union_find/disjoint_set_union.hpp"
#define CHECK(x) do{if(!(x)){std::cerr<<"check failed line "<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
std::mt19937 rng(310112026);int draw(int n){return int(rng()%n);}using G=cp::Graph<long long>;
int main(){
 for(int trial=0;trial<280;++trial){int n=draw(8);G g(n);std::vector<std::vector<bool>> adj(n,std::vector<bool>(n));for(int u=0;u<n;++u)for(int v=u+1;v<n;++v)if(draw(2)){g.add_edge(u,v);adj[u][v]=adj[v][u]=true;}auto p=cp::chromatic_polynomial<long long>(g);CHECK(p.size()==static_cast<std::size_t>(n+1));for(int q=0;q<=4;++q){long long count=0;std::vector<int> colors(n);std::function<void(int)> dfs=[&](int v){if(v==n){++count;return;}for(int c=0;c<q;++c){bool ok=true;for(int u=0;u<v;++u)if(adj[u][v] && colors[u]==c)ok=false;if(ok){colors[v]=c;dfs(v+1);}}};dfs(0);long long value=0;for(auto it=p.rbegin();it!=p.rend();++it)value=value*q+*it;CHECK(value==count);}
  long long cycles=0;for(int a=0;a<n;++a)for(int b=0;b<n;++b)for(int c=0;c<n;++c)for(int d=0;d<n;++d)if(a!=b && a!=c && a!=d && b!=c && b!=d && c!=d && adj[a][b] && adj[b][c] && adj[c][d] && adj[d][a])++cycles;CHECK(cp::count_four_cycles(g)==static_cast<unsigned long long>(cycles/8));
  std::set<unsigned> expected,actual;for(unsigned mask=1;mask<(1u<<n);++mask){bool clique=true,maximal=true;for(int v=0;v<n;++v){bool can=true;for(int u=0;u<n;++u)if((mask>>u&1) && u!=v && !adj[u][v])can=false;if(mask>>v&1){if(!can)clique=false;}else if(can)maximal=false;}if(clique && maximal)expected.insert(mask);}CHECK(cp::enumerate_maximal_cliques(g,[&](const auto& vertices){unsigned mask=0;for(int v:vertices)mask|=1u<<v;CHECK(actual.insert(mask).second);return true;}));CHECK(actual==expected);
  std::vector<int> permutation(n);std::iota(permutation.begin(),permutation.end(),0);std::shuffle(permutation.begin(),permutation.end(),rng);G h(n);for(auto e:g.edges())h.add_edge(permutation[e.from],permutation[e.to]);auto iso=cp::graph_isomorphism_filter(g,h);CHECK(iso.status!=cp::IsomorphismStatus::different);if(iso.status==cp::IsomorphismStatus::isomorphic){CHECK(iso.mapping.size()==static_cast<std::size_t>(n));auto sorted=iso.mapping;std::sort(sorted.begin(),sorted.end());for(int v=0;v<n;++v)CHECK(sorted[v]==v);for(int u=0;u<n;++u)for(int v=0;v<n;++v)CHECK(adj[u][v]==std::any_of(h[iso.mapping[u]].begin(),h[iso.mapping[u]].end(),[&](auto e){return e.to==iso.mapping[v];}));}
 }
 {G cycle(6),triangles(6);for(int v=0;v<6;++v)cycle.add_edge(v,(v+1)%6);for(int v=0;v<6;++v)triangles.add_edge(v,(v/3)*3+(v+1)%3);CHECK(cp::graph_isomorphism_filter(cycle,triangles).status==cp::IsomorphismStatus::inconclusive);G loop(1);loop.add_edge(0,0);CHECK(cp::chromatic_polynomial<long long>(loop)==std::vector<long long>({0,0}));}
 for(int trial=0;trial<550;++trial){int n=1+draw(7),m=draw(11);G g(n);for(int i=0;i<m;++i)g.add_edge(draw(n),draw(n),draw(8));std::vector<int> terminals;for(int i=0,k=draw(5);i<k;++i)terminals.push_back(draw(n));std::optional<long long> optimum;
  for(unsigned mask=0;mask<(1u<<m);++mask){cp::DisjointSetUnion dsu(n);long long cost=0;for(auto e:g.edges())if(mask>>e.idx&1){dsu.merge(e.from,e.to);cost+=e.cost;}bool ok=true;for(int v:terminals)if(!dsu.same(terminals[0],v))ok=false;if(ok && (!optimum || cost<*optimum))optimum=cost;}
  auto result=cp::steiner_tree(g,terminals);CHECK(bool(result)==bool(optimum));if(result){CHECK(result->cost==*optimum);cp::DisjointSetUnion dsu(n);long long cost=0;std::set<int> ids;for(int id:result->edge_ids){CHECK(ids.insert(id).second);auto e=g.edges()[id];CHECK(dsu.merge(e.from,e.to));cost+=e.cost;}CHECK(cost==result->cost);for(int v:terminals)CHECK(dsu.same(terminals[0],v));}
 }
 for(int trial=0;trial<350;++trial){int n=draw(8);G g(n,cp::GraphKind::Directed);for(int u=0;u<n;++u)for(int v=u+1;v<n;++v)if(draw(3)==0)g.add_edge(u,v);int max_matching=0;std::function<void(int,unsigned,int)> enumerate=[&](int v,unsigned used,int count){if(v==n){max_matching=std::max(max_matching,count);return;}enumerate(v+1,used,count);for(auto e:g[v])if(!(used>>e.to&1))enumerate(v+1,used|(1u<<e.to),count+1);};enumerate(0,0,0);auto result=cp::dag_minimum_path_cover(g);CHECK(result.size()==n-max_matching);std::vector<int> seen(n);for(int i=0;i<result.size();++i){auto path=result.paths[i];CHECK(path.size()==result.edge_ids[i].size()+1);for(int v:path)++seen[v];for(std::size_t j=0;j+1<path.size();++j){auto e=g.edges()[result.edge_ids[i][j]];CHECK(e.from==path[j] && e.to==path[j+1]);}}for(int x:seen)CHECK(x==1);
 }
 for(bool directed:{false,true})for(int trial=0;trial<300;++trial){int n=2+draw(5),m=draw(9);G g(n,directed?cp::GraphKind::Directed:cp::GraphKind::Undirected);for(int i=0;i<m;++i)g.add_edge(draw(n),draw(n));struct Path{unsigned edges,vertices;};std::vector<Path> paths;
  std::function<void(int,unsigned,unsigned)> enumerate=[&](int v,unsigned es,unsigned vs){if(v==n-1){paths.push_back({es,vs&~1u&~(1u<<(n-1))});return;}for(auto e:g[v])if(!(vs>>e.to&1))enumerate(e.to,es|(1u<<e.idx),vs|(1u<<e.to));};enumerate(0,0,1);
  for(bool vertex:{false,true}){std::function<int(std::size_t,unsigned,unsigned)> choose=[&](std::size_t i,unsigned es,unsigned vs){if(i==paths.size())return 0;int best=choose(i+1,es,vs);if(!(es&paths[i].edges) && (!vertex || !(vs&paths[i].vertices)))best=std::max(best,1+choose(i+1,es|paths[i].edges,vs|paths[i].vertices));return best;};int expected=choose(0,0,0);auto result=cp::disjoint_paths(g,0,n-1,vertex);CHECK(result.size()==expected);std::set<int> edge_seen,vertex_seen;for(int i=0;i<result.size();++i){auto path=result.paths[i];CHECK(path.front()==0 && path.back()==n-1);CHECK(path.size()==result.edge_ids[i].size()+1);for(std::size_t j=0;j+1<path.size();++j){int id=result.edge_ids[i][j];CHECK(edge_seen.insert(id).second);auto e=g.edges()[id];CHECK((e.from==path[j] && e.to==path[j+1])||(!directed && e.to==path[j] && e.from==path[j+1]));if(vertex && j>0)CHECK(vertex_seen.insert(path[j]).second);}}}
 }
 std::cout<<"combinatorics coloring/subset/path enumeration checks passed\n";
}
