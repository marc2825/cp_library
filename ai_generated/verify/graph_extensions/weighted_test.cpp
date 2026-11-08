#include "headers.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#define CHECK(x) do {if(!(x)){std::cerr<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
using namespace cp;
using LL=long long;using W=OptimizationWide;
std::mt19937_64 rng(871238);
int pick(int n){return int(rng()%n);}
template<class T> void witness(const Graph<T>& g,const std::vector<int>& vertices,const std::vector<int>& ids) {
    CHECK(vertices.size()==ids.size()+1);CHECK(vertices.front()==vertices.back());CHECK(!ids.empty());
    std::set<int> seen;for(int i=0;i<int(ids.size());++i) {
        CHECK(seen.insert(vertices[i]).second);auto e=g.edges().at(ids[i]);int u=vertices[i],v=vertices[i+1];
        CHECK((u==e.from && v==e.to)||(!g.directed() && u==e.to && v==e.from));
    }
}
void cycles() {
    for(int trial=0;trial<1800;++trial) {
        int n=1+pick(7);Graph<LL> g(n,GraphKind::Directed);DifferenceConstraints<LL> constraints(n);
        for(int i=0,m=pick(20);i<m;++i){int u=pick(n),v=pick(n),w=pick(19)-9;g.add_edge(u,v,w);constraints.add_constraint(u,v,w);}
        std::optional<std::pair<W,int>> best;
        for(int s=0;s<n;++s) {
            auto dfs=[&](auto&& self,int v,unsigned used,W cost,int length)->void {
                for(auto e:g[v]) {
                    if(e.to==s) {W a=cost+e.cost;int b=length+1;if(!best || a*best->second<best->first*b)best={{a,b}};}
                    else if(e.to>s && !(used>>e.to&1))self(self,e.to,used|(1u<<e.to),cost+e.cost,length+1);
                }
            };dfs(dfs,s,1u<<s,0,0);
        }
        auto mean=minimum_mean_cycle(g);CHECK(bool(mean)==bool(best));
        if(mean){CHECK(mean->numerator*best->second==best->first*mean->denominator);witness(g,mean->vertices,mean->edge_ids);W sum=0;for(int id:mean->edge_ids)sum+=g.edges()[id].cost;CHECK(sum*mean->denominator==mean->numerator*W(mean->edge_ids.size()));}
        auto neg=find_negative_cycle(g);CHECK(bool(neg)==(best && best->first<0));
        if(neg){witness(g,neg->vertices,neg->edge_ids);W sum=0;for(int id:neg->edge_ids)sum+=g.edges()[id].cost;CHECK(sum==neg->weight && sum<0);}
        auto solved=constraints.solve();CHECK(solved.feasible()==!neg);
        if(solved.feasible()){CHECK(int(solved.assignment.size())==n);for(auto e:g.edges())CHECK(solved.assignment[e.to]-solved.assignment[e.from]<=e.cost);}
        int source=pick(n);std::vector<bool> reached(n);reached[source]=true;for(int i=0;i<n;++i)for(auto e:g.edges())if(reached[e.from])reached[e.to]=true;
        Graph<LL> sub(n,GraphKind::Directed);for(auto e:g.edges())if(reached[e.from])sub.add_edge(e.from,e.to,e.cost);
        CHECK(bool(find_negative_cycle(g,source))==bool(find_negative_cycle(sub)));
    }
    Graph<LL> extreme(3,GraphKind::Directed);extreme.add_edge(0,1,std::numeric_limits<LL>::lowest());extreme.add_edge(1,2,std::numeric_limits<LL>::lowest());extreme.add_edge(2,0,std::numeric_limits<LL>::max());
    auto a=minimum_mean_cycle(extreme);CHECK(a && a->numerator*3==(W(std::numeric_limits<LL>::lowest())-1)*a->denominator);CHECK(find_negative_cycle(extreme));
    DifferenceConstraints<LL> chain(3);chain.add_constraint(0,1,std::numeric_limits<LL>::lowest());chain.add_constraint(1,2,std::numeric_limits<LL>::lowest());CHECK(chain.solve().assignment[2]==2*W(std::numeric_limits<LL>::lowest()));
    Graph<LL> undirected(2);undirected.add_edge(0,1,-1);auto cycle=find_negative_cycle(undirected);CHECK(cycle && cycle->edge_ids==std::vector<int>({0,0}));
    std::cout<<"1800 exact cycle/constraint comparisons passed\n";
}
std::vector<std::vector<LL>> distances(const Graph<LL>& g) {
    int n=g.size();std::vector<std::vector<LL>> d(n,std::vector<LL>(n,1000000000));
    for(int v=0;v<n;++v){d[v][v]=0;for(auto e:g[v])d[v][e.to]=std::min(d[v][e.to],e.cost);}
    for(int k=0;k<n;++k)for(int u=0;u<n;++u)for(int v=0;v<n;++v)d[u][v]=std::min(d[u][v],d[u][k]+d[k][v]);
    return d;
}
void segment() {
    for(int trial=0;trial<160;++trial) {
        int n=1+pick(12);SegmentTreeGraph<LL> builder(n);Graph<LL> explicit_graph(n,GraphKind::Directed);
        for(int q=0;q<40;++q) {
            int l=pick(n+1),r=pick(n+1),a=pick(n+1),b=pick(n+1);if(l>r)std::swap(l,r);if(a>b)std::swap(a,b);int mode=pick(4),u=pick(n),v=pick(n);LL cost=pick(10);int id=-1;
            if(mode==0){id=builder.add_edge(u,v,cost);l=u;r=u+1;a=v;b=v+1;}
            if(mode==1){id=builder.add_point_to_range(u,a,b,cost);l=u;r=u+1;}
            if(mode==2){id=builder.add_range_to_point(l,r,v,cost);a=v;b=v+1;}
            if(mode==3)id=builder.add_range_to_range(l,r,a,b,cost);
            CHECK(id==q);for(int x=l;x<r;++x)for(int y=a;y<b;++y)explicit_graph.add_edge(x,y,cost);
        }
        auto g=builder.graph();auto d=distances(g),expected=distances(explicit_graph);
        for(int u=0;u<n;++u)for(int v=0;v<n;++v)CHECK(d[u][v]==expected[u][v]);
        CHECK(builder.edge_query_ids().size()==g.edges().size());for(auto e:g.edges()){int q=builder.edge_query_ids()[e.idx];CHECK(q>=-1 && q<40);if(q<0)CHECK(e.cost==0);}
        auto snapshot=builder.graph();builder.add_edge(0,0,1);CHECK(builder.graph().edge_count()==snapshot.edge_count()+1);
    }
    SegmentTreeGraph<> empty(0);empty.add_range_to_range(0,0,0,0,5);CHECK(empty.graph().size()==0);
    std::cout<<"160 interval graph all-pairs comparisons passed\n";
}
template<class T> std::pair<W,int> kruskal(const Graph<T>& g) {
    auto edges=g.edges();std::sort(edges.begin(),edges.end(),[](auto a,auto b){return a.cost<b.cost;});DisjointSetUnion d(g.size());W total=0;
    for(auto e:edges)if(d.merge(e.from,e.to))total+=W(e.cost);
    return {total,d.component_count()};
}
void mst() {
    for(int trial=0;trial<170;++trial) {
        int n=1+pick(30);Graph<LL> g(n);IncrementalMSF<LL> online(n);
        for(int i=0;i<130;++i) {
            int u=pick(n),v=pick(n);LL weight=pick(100)-50;int id=g.add_edge(u,v,weight);auto update=online.add_edge(u,v,weight);CHECK(update.id==id);
            auto expected=kruskal(g);CHECK(online.cost()==expected.first && online.components()==expected.second);
            W total=0;DisjointSetUnion d(n);for(int selected:online.selected_edges()){auto e=g.edges()[selected];CHECK(d.merge(e.from,e.to));total+=e.cost;}CHECK(total==expected.first && d.component_count()==expected.second);
        }
        auto b=boruvka_msf(g);CHECK(b.cost==online.cost() && b.components==online.components());
        std::vector<std::uint64_t> values(n);for(auto& x:values)x=trial%2?rng():std::uint64_t(pick(16));
        Graph<std::uint64_t> complete(n);for(int u=0;u<n;++u)for(int v=u+1;v<n;++v)complete.add_edge(u,v,values[u]^values[v]);
        auto x=xor_minimum_spanning_tree(values);CHECK(x.cost==kruskal(complete).first);
        DisjointSetUnion d(n);W total=0;for(auto e:x.edges){CHECK(d.merge(e.from,e.to));CHECK(e.cost==(values[e.from]^values[e.to]));total+=e.cost;}CHECK(total==x.cost && x.components==1);
    }
    IncrementalMSF<LL> extreme(3);extreme.add_edge(0,1,std::numeric_limits<LL>::lowest());extreme.add_edge(1,2,std::numeric_limits<LL>::lowest());CHECK(extreme.cost()==2*W(std::numeric_limits<LL>::lowest()));
    CHECK(xor_minimum_spanning_tree({}).components==0);
    std::cout<<"22100 online MSF prefixes and XOR complete graphs passed\n";
}
void paths() {
    for(int trial=0;trial<1000;++trial) {
        int n=2+pick(5);Graph<LL> g(n,trial%2?GraphKind::Directed:GraphKind::Undirected);
        for(int m=pick(11);m--;){int u=pick(n),v=pick(n);g.add_edge(u,v,pick(9));}
        struct P{unsigned edges,inside;LL cost;};std::vector<P> paths;
        auto dfs=[&](auto&& self,int v,unsigned used,unsigned edges,LL cost)->void{
            if(v==n-1){paths.push_back({edges,used&~1u&~(1u<<(n-1)),cost});return;}
            for(auto e:g[v])if(!(used>>e.to&1))self(self,e.to,used|(1u<<e.to),edges|(1u<<e.idx),cost+e.cost);
        };dfs(dfs,0,1,0,0);
        for(bool vertex:{false,true})for(int k=0;k<=3;++k) {
            LL best=1000000;
            auto choose=[&](auto&& self,int pos,int left,unsigned edges,unsigned inside,LL cost)->void{
                if(!left){best=std::min(best,cost);return;}
                for(int i=pos;i<int(paths.size());++i)if(!(edges&paths[i].edges) && (!vertex || !(inside&paths[i].inside)))
                    self(self,i+1,left-1,edges|paths[i].edges,inside|paths[i].inside,cost+paths[i].cost);
            };choose(choose,0,k,0,0,0);
            auto r=min_cost_disjoint_paths(g,0,n-1,k,vertex);CHECK(bool(r)==(best<1000000));
            if(!r)continue;
            CHECK(r->cost==best && r->size()==k);unsigned used_edges=0,inside=0;LL total=0;
            for(int i=0;i<k;++i) {
                auto p=r->paths[i];CHECK(p.front()==0 && p.back()==n-1);CHECK(p.size()==r->edge_ids[i].size()+1);std::set<int> unique(p.begin(),p.end());CHECK(unique.size()==p.size());
                for(int j=0;j<int(r->edge_ids[i].size());++j){int id=r->edge_ids[i][j];CHECK(!(used_edges>>id&1));used_edges|=1u<<id;auto e=g.edges()[id];CHECK((e.from==p[j]&&e.to==p[j+1])||(!g.directed()&&e.to==p[j]&&e.from==p[j+1]));total+=e.cost;}
                if(vertex)for(int j=1;j+1<int(p.size());++j){CHECK(!(inside>>p[j]&1));inside|=1u<<p[j];}
            }
            CHECK(total==best);
        }
    }
    std::cout<<"8000 min-cost disjoint path enumerations passed\n";
}
int main(){cycles();segment();mst();paths();}
