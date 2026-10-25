#include "common.hpp"
#include <array>
#include "graph_basic/few_weight_shortest_path.hpp"
#include "graph_basic/dijkstra.hpp"
#include "graph_basic/shortest_nonzero_path.hpp"
#include "graph_basic/minimum_diameter_spanning_tree.hpp"
using namespace cp;
struct Mod3 { using value_type=int; static int identity(){return 0;} static int op(int a,int b){return (a+b)%3;} static int inverse(int a){return (3-a)%3;} };
struct S3 {
    using value_type=std::array<int,3>;
    static value_type identity(){return {0,1,2};}
    static value_type op(value_type a,value_type b){return {b[a[0]],b[a[1]],b[a[2]]};}
    static value_type inverse(value_type a){value_type b{};for(int i=0;i<3;++i)b[a[i]]=i;return b;}
};
template<class Group> void nonzero(std::mt19937& rng,const std::vector<typename Group::value_type>& alphabet) {
    using L=typename Group::value_type;
    for(int trial=0;trial<1600;++trial) {
        int n=1+int(rng()%8); Graph<int> g(n); std::vector<L> labels;
        for(int i=0;i<12;++i) { g.add_edge(int(rng()%n),int(rng()%n),int(rng()%7)); labels.push_back(alphabet[rng()%alphabet.size()]); }
        int source=int(rng()%n); auto actual=shortest_nonzero_paths<Group>(g,labels,source); std::vector<std::optional<int>> best(n);
        std::function<void(int,int,int,L)> dfs=[&](int v,int seen,int d,L label) {
            if(!(label==Group::identity()) && (!best[v] || d<*best[v])) best[v]=d;
            for(auto e:g[v]) if(!(seen>>e.to&1)) { L a=g.edges()[e.idx].from==v?labels[e.idx]:Group::inverse(labels[e.idx]); dfs(e.to,seen|(1<<e.to),d+e.cost,Group::op(label,a)); }
        }; dfs(source,1<<source,0,Group::identity());
        for(int v=0;v<n;++v) {
            CHECK(bool(actual.distance[v])==bool(best[v])); auto path=actual.path(v);
            if(!best[v]) { CHECK(path.vertices.empty()); continue; }
            CHECK(*actual.distance[v]==*best[v]); CHECK(path.vertices.front()==source && path.vertices.back()==v); CHECK(path.vertices.size()==path.edge_ids.size()+1);
            CHECK(std::set<int>(path.vertices.begin(),path.vertices.end()).size()==path.vertices.size()); int d=0; L label=Group::identity();
            for(int j=0;j<int(path.edge_ids.size());++j) { int id=path.edge_ids[j],a=path.vertices[j],b=path.vertices[j+1]; auto e=g.edges()[id]; CHECK((e.from==a && e.to==b)||(e.from==b && e.to==a)); d+=e.cost; label=Group::op(label,e.from==a?labels[id]:Group::inverse(labels[id])); }
            CHECK(d==*best[v] && !(label==Group::identity()));
        }
    }
}
int main() {
    std::mt19937 rng(90731);
    for(int trial=0;trial<500;++trial) {
        int n=1+int(rng()%35); Graph<long long> g(n,trial%2?GraphKind::Directed:GraphKind::Undirected);
        const long long palette[]={0,1,1000000000000LL};
        for(int i=0;i<100;++i) g.add_edge(int(rng()%n),int(rng()%n),palette[rng()%3]);
        int s=int(rng()%n); CHECK(few_weight_shortest_path(g,s).dist==dijkstra(g,s).dist);
    }
    nonzero<Mod3>(rng,{0,1,2}); std::vector<S3::value_type> permutations; auto p=S3::identity(); do {permutations.push_back(p);} while(std::next_permutation(p.begin(),p.end())); nonzero<S3>(rng,permutations);
    for(int trial=0;trial<500;++trial) {
        int n=1+int(rng()%6),m=int(rng()%11); Graph<int> g(n); for(int i=0;i<m;++i) g.add_edge(int(rng()%n),int(rng()%n),int(rng()%8));
        std::optional<int> best;
        for(int mask=0;mask<(1<<m);++mask) if(__builtin_popcount(unsigned(mask))==n-1) {
            std::vector<std::vector<int>> d(n,std::vector<int>(n,100000)); for(int v=0;v<n;++v)d[v][v]=0;
            for(auto e:g.edges()) if(mask>>e.idx&1) d[e.from][e.to]=d[e.to][e.from]=std::min(d[e.from][e.to],e.cost);
            for(int k=0;k<n;++k) for(int i=0;i<n;++i) for(int j=0;j<n;++j) d[i][j]=std::min(d[i][j],d[i][k]+d[k][j]);
            int diameter=0; for(auto row:d) for(int x:row) diameter=std::max(diameter,x);
            if(diameter<100000 && (!best || diameter<*best)) best=diameter;
        }
        auto result=minimum_diameter_spanning_tree(g); CHECK(bool(result)==bool(best));
        if(result) { CHECK(result->diameter==*best); CHECK(result->radius_twice==*best); CHECK(int(result->edge_ids.size())==n-1); }
    }
    Graph<long long> edge(2); edge.add_edge(0,1,LLONG_MAX); CHECK(minimum_diameter_spanning_tree(edge)->diameter==LLONG_MAX);
    CHECK(minimum_diameter_spanning_tree(Graph<int>(0))->diameter==0);
    throws<std::invalid_argument>([]{Graph<int> g(2);g.add_edge(0,1,-1);few_weight_shortest_path(g);});
    Graph<int> chain(20000); std::vector<int> labels;
    for(int v=1;v<chain.size();++v) { chain.add_edge(v-1,v,1); labels.push_back(1); }
    auto long_path=shortest_nonzero_paths<Mod3>(chain,labels,0);
    CHECK(*long_path.distance.back()==19999 && long_path.path(19999).edge_ids.size()==19999);
    CHECK(few_weight_shortest_path(chain).dist.back()==19999);
    Graph<int> detour(5);detour.add_edge(0,1,1);detour.add_edge(1,2,1);detour.add_edge(1,3,1);detour.add_edge(3,4,1);detour.add_edge(4,1,1);
    CHECK(!shortest_nonzero_paths<Mod3>(detour,{0,0,1,0,0},0).distance[2]);
    std::cout<<"paths: 500 queue comparisons, 3200 group-labeled simple-path enumerations (including noncommutative S3), 500 spanning-tree enumerations passed\n";
}
