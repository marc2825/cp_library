#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>
#include "graph_connectivity/all.hpp"
#include "tree/rooted_tree.hpp"
void check(bool ok){if(!ok)throw std::runtime_error("graph structure mismatch");}
int main(){
    for(int n=0;n<=5;++n){
        int count=1;for(int i=0;i<n;++i)count*=n;
        for(int code=0;code<count;++code){std::vector<int>a(n);int c=code;for(int& x:a){x=c%n;c/=n;}cp::FunctionalGraph f(a);
            for(int u=0;u<n;++u){std::vector<int>seen(n,-1),walk;int v=u;while(seen[v]<0){seen[v]=int(walk.size());walk.push_back(v);v=a[v];}int depth=seen[v],length=int(walk.size())-depth;check(f.depth[u]==depth&&f.entry[u]==v&&int(f.cycles[f.component[u]].size())==length);
                for(int t=0;t<n;++t){auto d=f.min_steps(u,t);check(bool(d)==(seen[t]>=0));if(d)check(*d==std::uint64_t(seen[t]));}
                for(auto steps:{UINT64_C(0),UINT64_C(1),UINT64_C(50),UINT64_MAX}){int index=steps<walk.size()?int(steps):depth+int((steps-depth)%length);check(f.jump(u,steps)==walk[index]);}
            }
            for(const auto& cycle:f.cycles)for(int i=0;i<int(cycle.size());++i)check(a[cycle[i]]==cycle[(i+1)%cycle.size()]);
        }
    }
    for(int mask=0;mask<1024;++mask){cp::TwoSAT sat(2);std::vector<std::array<int,4>>clauses;int index=0;for(int a=0;a<4;++a)for(int b=a;b<4;++b){if(mask>>index&1){sat.add_clause(a/2,a%2,b/2,b%2);clauses.push_back({a/2,a%2,b/2,b%2});}++index;}
        auto answer=sat.solve();bool exists=false;for(int x=0;x<4;++x){bool ok=true;for(auto c:clauses)ok=ok&&(((x>>c[0]&1)==c[1])||((x>>c[2]&1)==c[3]));exists|=ok;}check(bool(answer)==exists);if(answer)for(auto c:clauses)check((*answer)[c[0]]==bool(c[1])||(*answer)[c[2]]==bool(c[3]));
    }
    cp::TwoSAT sat(0);check(sat.solve()->empty());cp::TwoSAT unit(1);unit.force(0,true);check((*unit.solve())[0]);unit.force(0,false);check(!unit.solve());
    std::mt19937 rng(89123);
    for(int trial=0;trial<750;++trial){
        int n=1+rng()%6,m=rng()%9;bool directed=rng()%2;cp::Graph<int>g(n,directed?cp::GraphKind::Directed:cp::GraphKind::Undirected);for(int i=0;i<m;++i)g.add_edge(rng()%n,rng()%n);
        for(int start=-1;start<n;++start){
            auto brute=[&](auto&& self,int v,int mask)->bool{if(mask==(1<<m)-1)return true;for(auto e:g[v])if(!(mask>>e.idx&1)&&self(self,e.to,mask|(1<<e.idx)))return true;return false;};bool exists=false;if(start==-1){for(int s=0;s<n;++s)exists=exists||brute(brute,s,0);}else exists=brute(brute,start,0);
            auto trail=cp::eulerian_trail(g,start);check(bool(trail)==exists);if(trail){check(trail->edge_ids.size()==std::size_t(m)&&trail->vertices.size()==std::size_t(m+1));if(start!=-1)check(trail->vertices.front()==start);std::vector<bool>used(m);for(int i=0;i<m;++i){int id=trail->edge_ids[i];check(!used[id]);used[id]=true;auto e=g.edges()[id];int u=trail->vertices[i],v=trail->vertices[i+1];check((e.from==u&&e.to==v)||(!directed&&e.from==v&&e.to==u));}}
        }
        if(!directed&&n<=5){
            std::vector<std::vector<bool>>adj(n,std::vector<bool>(n));for(auto e:g.edges())if(e.from!=e.to)adj[e.from][e.to]=adj[e.to][e.from]=true;
            auto connected=[&](int mask){if(!mask)return true;int s=0;while(!(mask>>s&1))++s;int seen=1<<s;std::vector<int>q{s};for(std::size_t i=0;i<q.size();++i)for(int v=0;v<n;++v)if((mask>>v&1)&&!(seen>>v&1)&&adj[q[i]][v]){seen|=1<<v;q.push_back(v);}return seen==mask;};
            std::vector<int>valid;for(int mask=1;mask<(1<<n);++mask){if(!connected(mask))continue;bool ok=true;for(int v=0;v<n;++v)if(mask>>v&1)ok=ok&&connected(mask^(1<<v));if(ok)valid.push_back(mask);}
            std::set<int>expected;for(int x:valid){bool maximal=true;for(int y:valid)if(x!=y&&(x&y)==x)maximal=false;if(maximal)expected.insert(x);}
            auto blocks=cp::block_cut_forest(g);std::set<int>actual;for(auto b:blocks.blocks){int mask=0;for(int v:b)mask|=1<<v;actual.insert(mask);}check(actual==expected);cp::RootedTree<int>forest(blocks.forest);check(forest.size()==n+int(blocks.blocks.size()));
        }
    }
    cp::Graph<int>empty(0);check(cp::eulerian_trail(empty)->vertices.empty());
    cp::Graph<int>large(200000,cp::GraphKind::Directed);for(int v=1;v<large.size();++v)large.add_edge(v-1,v);check(cp::eulerian_trail(large)->edge_ids.size()==199999);
    cp::TwoSAT chain(200000);chain.force(0,true);for(int v=1;v<chain.size();++v)chain.imply(v-1,true,v,true);check(chain.solve()->back());
    std::cout<<"structure: all functional maps n<=5, 1024 SAT formulas, 750 Euler multigraphs, independent blocks and 200000 vertices passed\n";
}
