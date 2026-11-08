#include "headers.hpp"
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#define CHECK(x) do { if(!(x)) {std::cerr << __LINE__ << ": " << #x << '\n'; std::abort();} } while(false)
using namespace cp;
int bits(unsigned x) {return __builtin_popcount(x);}
bool possible_st(const Graph<int>& g,int s,int t) {
    std::vector<int> mid;for(int v=0;v<g.size();++v)if(v!=s && v!=t)mid.push_back(v);
    do {
        std::vector<int> rank(g.size());rank[s]=0;rank[t]=g.size()-1;
        for(int i=0;i<int(mid.size());++i)rank[mid[i]]=i+1;
        bool ok=true;
        for(int v=0;v<g.size();++v) {
            bool lo=v==s,hi=v==t;for(auto e:g[v]) {lo=lo || rank[e.to]<rank[v];hi=hi || rank[e.to]>rank[v];}
            ok=ok && lo && hi;
        }
        if(ok)return true;
    }while(std::next_permutation(mid.begin(),mid.end()));return false;
}
void undirected() {
    long long count=0;
    for(int n=0;n<=5;++n) {
        std::vector<std::pair<int,int>> pairs;for(int u=0;u<n;++u)for(int v=u+1;v<n;++v)pairs.emplace_back(u,v);
        std::set<std::vector<int>> graphical;
        for(unsigned mask=0;mask<(1u<<pairs.size());++mask) {
            Graph<int> g(n);std::vector<int> degree(n);
            for(int i=0;i<int(pairs.size());++i)if(mask>>i&1) {auto [u,v]=pairs[i];g.add_edge(u,v,1+(i*7)%5);++degree[u];++degree[v];}
            graphical.insert(degree);
            auto coloring=vizing_edge_coloring(g);CHECK(coloring.color_count<=coloring.max_degree+1);
            for(int v=0;v<n;++v){std::set<int> colors;for(auto e:g[v])CHECK(colors.insert(coloring.color[e.idx]).second);}
            auto cover=minimum_edge_cover(g);int best=n+1;
            for(unsigned chosen=0;chosen<(1u<<g.edge_count());++chosen) {
                unsigned seen=0;for(auto e:g.edges())if(chosen>>e.idx&1)seen|=1u<<e.from|1u<<e.to;
                if(seen==(1u<<n)-1)best=std::min(best,bits(chosen));
            }
            CHECK(bool(cover)==(best<=n));
            if(cover){CHECK(int(cover->size())==best);unsigned seen=0;for(int id:*cover){auto e=g.edges()[id];seen|=1u<<e.from|1u<<e.to;}CHECK(seen==(1u<<n)-1);}
            auto tree=gomory_hu_tree(g);CHECK(tree.edge_count()==std::max(0,n-1));
            for(int s=0;s<n;++s)for(int t=s+1;t<n;++t) {
                int cut=100000;
                for(unsigned side=0;side<(1u<<n);++side)if((side>>s&1) && !(side>>t&1)) {
                    int cap=0;for(auto e:g.edges())if(((side>>e.from)^(side>>e.to))&1)cap+=e.cost;cut=std::min(cut,cap);
                }
                std::vector<int> val(n,-1),queue{s};val[s]=100000;
                for(std::size_t i=0;i<queue.size();++i)for(auto e:tree[queue[i]])if(val[e.to]<0){val[e.to]=std::min(val[e.from],e.cost);queue.push_back(e.to);}
                CHECK(val[t]==cut);
                auto st=st_numbering(g,s,t);CHECK(bool(st)==possible_st(g,s,t));
                if(st) {CHECK(st->order.front()==s && st->order.back()==t);for(auto [u,v]:st->oriented_edges)CHECK(st->rank[u]<st->rank[v]);}
            }
            ++count;
        }
        std::vector<int> d(n);
        auto enumerate=[&](auto&& self,int i)->void {
            if(i==n) {
                bool yes=graphical.count(d);CHECK(is_graphical_degree_sequence(d)==yes);
                auto g=realize_degree_sequence(d);CHECK(bool(g)==yes);
                if(g) {std::vector<int> got(n);std::set<std::pair<int,int>> used;for(auto e:g->edges()){CHECK(e.from!=e.to);CHECK(used.insert(std::minmax(e.from,e.to)).second);++got[e.from];++got[e.to];}CHECK(got==d);}
                return;
            }
            for(int x=0;x<n;++x){d[i]=x;self(self,i+1);}
        };enumerate(enumerate,0);
    }
    CHECK(!is_graphical_degree_sequence({-1}));CHECK(!realize_degree_sequence({2,2}));
    std::cout<<count<<" undirected graphs / degree sequences passed\n";
}
void bipartite_and_dag() {
    for(int l=0;l<=3;++l)for(int r=0;r<=3;++r)for(unsigned mask=0;mask<(1u<<(l*r));++mask) {
        std::vector<std::pair<int,int>> edges;for(int u=0;u<l;++u)for(int v=0;v<r;++v)if(mask>>(u*r+v)&1)edges.emplace_back(u,v);
        auto c=bipartite_vertex_cover(l,r,edges);int best=l+r;
        for(unsigned chosen=0;chosen<(1u<<(l+r));++chosen) {
            bool ok=true;for(auto [u,v]:edges)if(!(chosen>>u&1) && !(chosen>>(l+v)&1))ok=false;
            if(ok)best=std::min(best,bits(chosen));
        }
        CHECK(int(c.left_cover.size()+c.right_cover.size())==best);
        CHECK(int(c.left_independent.size()+c.right_independent.size())==l+r-best);
        std::vector<bool> covered(l+r);for(int v:c.left_cover)covered[v]=true;for(int v:c.right_cover)covered[l+v]=true;
        for(auto [u,v]:edges)CHECK(covered[u] || covered[l+v]);
        auto edge_cover=bipartite_minimum_edge_cover(l,r,edges);
        std::vector<bool> incident(l+r);for(auto [u,v]:edges)incident[u]=incident[l+v]=true;
        CHECK(bool(edge_cover)==(std::find(incident.begin(),incident.end(),false)==incident.end()));
        if(edge_cover) {CHECK(int(edge_cover->size())==l+r-best);std::fill(incident.begin(),incident.end(),false);for(int id:*edge_cover){auto [u,v]=edges[id];incident[u]=incident[l+v]=true;}CHECK(std::find(incident.begin(),incident.end(),false)==incident.end());}
    }
    for(int n=0;n<=5;++n) {
        int m=n*(n-1)/2;
        for(unsigned mask=0;mask<(1u<<m);++mask) {
            Graph<int> g(n,GraphKind::Directed);int bit=0;std::vector<std::vector<bool>> reach(n,std::vector<bool>(n));
            for(int u=0;u<n;++u)for(int v=u+1;v<n;++v)if(mask>>(bit++)&1){g.add_edge(u,v);reach[u][v]=true;}
            for(int k=0;k<n;++k)for(int u=0;u<n;++u)for(int v=0;v<n;++v)reach[u][v]=reach[u][v] || (reach[u][k] && reach[k][v]);
            int best=0;for(unsigned chosen=0;chosen<(1u<<n);++chosen){bool ok=true;for(int u=0;u<n;++u)for(int v=0;v<n;++v)if((chosen>>u&1)&&(chosen>>v&1)&&reach[u][v])ok=false;if(ok)best=std::max(best,bits(chosen));}
            auto a=dag_maximum_antichain(g);CHECK(int(a.antichain.size())==best);CHECK(int(a.chains.size())==best);
            for(int u:a.antichain)for(int v:a.antichain)CHECK(!reach[u][v]);
            std::vector<int> seen(n);for(auto chain:a.chains){for(int v:chain)++seen[v];for(int i=1;i<int(chain.size());++i)CHECK(reach[chain[i-1]][chain[i]]);}
            for(int x:seen)CHECK(x==1);
        }
    }
    std::cout<<"bipartite and DAG exhaustive checks passed\n";
}
void unicyclic() {
    std::mt19937 rng(149);
    for(int n=1;n<=100;++n)for(int trial=0;trial<15;++trial) {
        Graph<int> g(n);for(int v=1;v<n;++v)g.add_edge(v,int(rng()%v));g.add_edge(int(rng()%n),int(rng()%n));
        auto a=unicyclic_decomposition(g);CHECK(a);CHECK(a->cycle.size()==a->cycle_edge_ids.size()+1);CHECK(a->cycle.front()==a->cycle.back());
        std::vector<int> vertices(n),edges(n);
        for(int i=0;i<int(a->cycle_edge_ids.size());++i){auto e=g.edges()[a->cycle_edge_ids[i]];int u=a->cycle[i],v=a->cycle[i+1];CHECK((e.from==u&&e.to==v)||(e.to==u&&e.from==v));++edges[e.idx];}
        for(int i=0;i<int(a->trees.size());++i)for(int v:a->trees[i]) {++vertices[v];CHECK(a->root[v]==a->cycle[i]);if(a->parent[v]>=0){++edges[a->parent_edge[v]];CHECK(a->depth[v]==a->depth[a->parent[v]]+1);}else CHECK(v==a->root[v]);}
        for(int x:vertices)CHECK(x==1);
        for(int x:edges)CHECK(x==1);
    }
    Graph<int> disconnected(3);disconnected.add_edge(0,0);disconnected.add_edge(1,1);disconnected.add_edge(1,2);CHECK(!unicyclic_decomposition(disconnected));
    Graph<int> empty;CHECK(!unicyclic_decomposition(empty));
    std::cout<<"unicyclic multigraph restoration passed\n";
}
int main(){undirected();bipartite_and_dag();unicyclic();}
