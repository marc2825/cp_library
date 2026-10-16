#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <utility>
#include <vector>
#include "graph_connectivity/all.hpp"
#include "graph/all.hpp"
#include "tree/all.hpp"

#define CHECK(x) do { if (!(x)) { std::cerr<<"line "<<__LINE__<<": " #x "\n"; std::abort(); } } while(false)
using G=cp::Graph<long long>;

template<class T> std::vector<std::vector<bool>> reachability(const cp::Graph<T>& g) {
    std::vector<std::vector<bool>> reach(g.size(),std::vector<bool>(g.size(),false));
    for(int s=0;s<g.size();++s){
        std::vector<int> queue{s};reach[s][s]=true;
        for(int p=0;p<int(queue.size());++p)for(auto e:g[queue[p]])if(!reach[s][e.to]){reach[s][e.to]=true;queue.push_back(e.to);}
    }
    return reach;
}
std::vector<int> labels(const G& g,int removed_vertex=-1,int removed_edge=-1) {
    std::vector<int> result(g.size(),-1);int count=0;
    for(int s=0;s<g.size();++s)if(s!=removed_vertex&&result[s]<0){
        std::vector<int> queue{s};result[s]=count++;
        for(int p=0;p<int(queue.size());++p)for(auto e:g[queue[p]])if(e.to!=removed_vertex&&e.idx!=removed_edge&&result[e.to]<0){result[e.to]=result[s];queue.push_back(e.to);}
    }
    return result;
}
int group_count(const std::vector<int>& labels){return labels.empty()?0:*std::max_element(labels.begin(),labels.end())+1;}

void check_scc(const G& g) {
    const auto reach=reachability(g);
    const auto result=cp::scc(g),named=cp::strongly_connected_components(g);
    CHECK(result.component==named.component&&result.groups==named.groups);
    std::vector<int> count(g.size(),0);
    for(int c=0;c<result.size();++c){
        CHECK(!result.groups[c].empty()&&std::is_sorted(result.groups[c].begin(),result.groups[c].end()));
        for(int v:result.groups[c]){CHECK(result.component[v]==c);++count[v];}
    }
    for(int x:count)CHECK(x==1);
    for(int u=0;u<g.size();++u)for(int v=0;v<g.size();++v)CHECK(result.same(u,v)==(reach[u][v]&&reach[v][u]));
    for(auto e:g.edges())CHECK(result.component[e.from]==result.component[e.to]||result.component[e.from]<result.component[e.to]);
    for(bool unique:{false,true}){
        const auto condensed=cp::condensation_graph(g,unique);
        CHECK(condensed.scc.component==result.component&&condensed.dag.directed()&&condensed.dag.size()==result.size());
        CHECK(int(condensed.original_edge.size())==condensed.dag.edge_count());
        std::multiset<std::pair<int,int>> expected,actual;
        for(auto e:g.edges())if(!result.same(e.from,e.to))expected.emplace(result.component[e.from],result.component[e.to]);
        if(unique){std::set<std::pair<int,int>> pairs(expected.begin(),expected.end());expected={pairs.begin(),pairs.end()};}
        for(auto e:condensed.dag.edges()){
            CHECK(e.from<e.to&&e.cost==1);
            auto original=g.edges()[condensed.original_edge[e.idx]];
            CHECK(result.component[original.from]==e.from&&result.component[original.to]==e.to);
            actual.emplace(e.from,e.to);
        }
        CHECK(actual==expected);
        auto compressed_reach=reachability(condensed.dag);
        for(int u=0;u<g.size();++u)for(int v=0;v<g.size();++v)CHECK(reach[u][v]==compressed_reach[result.component[u]][result.component[v]]);
    }
}
void check_undirected(const G& g) {
    auto original=labels(g);int count=group_count(original);
    const auto cc=cp::connected_components(g);CHECK(cc.size()==count);
    for(int u=0;u<g.size();++u)for(int v=0;v<g.size();++v)CHECK(cc.same(u,v)==(original[u]==original[v]));
    std::vector<int> bridges,cuts;
    for(int id=0;id<g.edge_count();++id)if(group_count(labels(g,-1,id))>count)bridges.push_back(id);
    for(int v=0;v<g.size();++v)if(group_count(labels(g,v))>count)cuts.push_back(v);
    auto low=cp::low_link(g);
    CHECK(low.bridge_edges==bridges&&cp::bridges(g)==bridges);
    CHECK(low.articulation==cuts&&cp::articulation_points(g)==cuts);
    auto ord=low.ord;std::sort(ord.begin(),ord.end());for(int v=0;v<g.size();++v)CHECK(ord[v]==v);
    auto ancestor=[&](int v,int u){for(int x=u;x!=-1;x=low.parent[x])if(x==v)return true;return false;};
    for(int v=0;v<g.size();++v){
        int expected_low=low.ord[v],subtree=0;
        if(low.parent[v]>=0){auto e=g.edges()[low.parent_edge[v]];CHECK((e.from==v&&e.to==low.parent[v])||(e.to==v&&e.from==low.parent[v]));CHECK(low.ord[low.parent[v]]<low.ord[v]);}
        for(int u=0;u<g.size();++u)if(ancestor(v,u)){
            ++subtree;
            for(auto e:g[u]){
                if(e.idx==low.parent_edge[u]||(low.parent[e.to]==u&&low.parent_edge[e.to]==e.idx))continue;
                expected_low=std::min(expected_low,low.ord[e.to]);
            }
        }
        CHECK(low.subtree_size[v]==subtree&&low.low[v]==expected_low);
    }
    bool possible=false;
    for(int mask=0;mask<(1<<g.size());++mask){bool valid=true;for(auto e:g.edges())valid&=((mask>>e.from)&1)!=((mask>>e.to)&1);possible|=valid;}
    auto color=cp::bipartite_coloring(g);CHECK(color.has_value()==possible);
    if(color){CHECK(int(color->size())==g.size());for(int c:*color)CHECK(c==0||c==1);for(auto e:g.edges())CHECK((*color)[e.from]!=(*color)[e.to]);}
}

void exhaustive(){
    int directed_count=0,undirected_count=0;
    for(int n=0;n<=3;++n){
        for(int mask=0;mask<(1<<(n*n));++mask){G g(n,cp::GraphKind::Directed);for(int u=0;u<n;++u)for(int v=0;v<n;++v)if(mask>>(u*n+v)&1)g.add_edge(u,v);check_scc(g);++directed_count;}
        std::vector<std::pair<int,int>> pairs;for(int u=0;u<n;++u)for(int v=u;v<n;++v)pairs.emplace_back(u,v);
        int limit=1;for(int i=0;i<int(pairs.size());++i)limit*=3;
        for(int code=0;code<limit;++code){G g(n);int value=code;for(auto [u,v]:pairs){int multiplicity=value%3;value/=3;for(int i=0;i<multiplicity;++i)g.add_edge(u,v);}check_undirected(g);++undirected_count;}
    }
    std::cout<<"exhaustive: "<<directed_count<<" directed and "<<undirected_count<<" undirected multigraphs passed\n";
}
void random_graphs(){
    std::mt19937 rng(38001);
    for(int trial=0;trial<600;++trial){
        int n=rng()%9;G g(n,trial%2?cp::GraphKind::Directed:cp::GraphKind::Undirected);
        if(n)for(int i=0,m=rng()%20;i<m;++i)g.add_edge(rng()%n,rng()%n,static_cast<long long>(rng()%101)-50);
        if(g.directed())check_scc(g);else check_undirected(g);
    }
    std::cout<<"600 random graphs: mutual reachability, DAG multiplicity/mapping, edge/vertex deletion, low/subtree and exhaustive 2-coloring passed\n";
}
struct OpaqueCost {int value;explicit OpaqueCost(int v):value(v){}};
void opaque_and_snapshot(){
    cp::Graph<OpaqueCost> directed(2,cp::GraphKind::Directed);directed.add_edge(0,1,OpaqueCost(-7));directed.add_edge(1,0,OpaqueCost(9));
    CHECK(cp::scc(directed).size()==1&&cp::condensation_graph(directed).dag.edge_count()==0);
    cp::Graph<OpaqueCost> undirected(2);undirected.add_edge(0,1,OpaqueCost(-3));
    CHECK(cp::low_link(undirected).bridge_edges==std::vector<int>{0}&&cp::bipartite_coloring(undirected));
    G g(2,cp::GraphKind::Directed);g.add_edge(0,1);
    auto before=cp::scc(g);g.add_edge(1,0);CHECK(before.size()==2&&cp::scc(g).size()==1);
}
void large(){
    constexpr int n=200000;
    {
        G g(n,cp::GraphKind::Directed);for(int i=0;i<n-1;++i)g.add_edge(i,i+1);
        auto result=cp::scc(g);CHECK(result.size()==n);for(int i=0;i<n;++i)CHECK(result.component[i]==i);
        auto dag=cp::condensation_graph(g);CHECK(dag.dag.edge_count()==n-1);
        g.add_edge(n-1,0);CHECK(cp::scc(g).size()==1);
    }
    {
        G g(n);for(int i=0;i<n-1;++i)g.add_edge(i,i+1);
        auto result=cp::low_link(g);CHECK(int(result.bridge_edges.size())==n-1&&int(result.articulation.size())==n-2);
        CHECK(cp::bipartite_coloring(g)&&cp::connected_components(g).size()==1);
        g.add_edge(0,2);CHECK(!cp::bipartite_coloring(g));
    }
    std::cout<<"200000 vertices: SCC chain/cycle, condensation, iterative Lowlink, components and bipartite passed\n";
}
int main(){exhaustive();random_graphs();opaque_and_snapshot();large();}
