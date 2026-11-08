#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <vector>
#include "../../tree/tree_center.hpp"
#include "../../tree/prufer_code.hpp"
#include "../../tree/rerooted_subtree.hpp"
#define CHECK(x) do { if (!(x)) { std::cerr<<__LINE__<<": "<<#x<<'\n';std::abort();} }while(false)
template<class F> void rejects(F f){bool threw=false;try{f();}catch(const std::invalid_argument&){threw=true;}CHECK(threw);}
template<class T> std::vector<T> distances(const cp::Graph<T>& g,int root){
    std::vector<T>d(g.size());std::vector<int>p(g.size(),-2),q{root};p[root]=-1;
    for(std::size_t i=0;i<q.size();++i)for(auto e:g[q[i]])if(p[e.to]==-2){p[e.to]=q[i];d[e.to]=d[q[i]]+e.cost;q.push_back(e.to);}
    return d;
}
void centers(){
    CHECK(cp::tree_center(cp::Graph<int>()).vertices.empty());
    CHECK(cp::weighted_tree_center(cp::Graph<int>()).vertex==-1);
    std::mt19937 rng(1732);
    for(int n=1;n<=70;++n)for(int rep=0;rep<12;++rep){
        cp::Graph<long long>g(n);cp::Graph<int>unit(n);
        for(int v=1;v<n;++v){int u=rng()%v;g.add_edge(u,v,rng()%15);unit.add_edge(u,v);}
        auto c=cp::weighted_tree_center(g);auto plain=cp::tree_center(g);
        std::vector<int>best,unweighted;long long radius=100000;int r=100000;
        for(int v=0;v<n;++v){auto d=distances(g,v);long long mx=*std::max_element(d.begin(),d.end());if(mx<radius){radius=mx;best.clear();}if(mx==radius)best.push_back(v);
            auto u=distances(unit,v);int m=*std::max_element(u.begin(),u.end());if(m<r){r=m;unweighted.clear();}if(m==r)unweighted.push_back(v);}
        CHECK(best==c.vertices && radius==c.vertex_radius);CHECK(plain.vertices==unweighted && plain.radius==r);
        long long maximum=0;
        if(c.vertex>=0){auto d=distances(g,c.vertex);for(auto x:d)maximum=std::max(maximum,2*x);}
        else {CHECK(c.edge_id>=0);auto da=distances(g,c.from),db=distances(g,c.to);long long length=g.edges()[c.edge_id].cost;CHECK(0<c.offset_twice && c.offset_twice<2*length);for(int v=0;v<n;++v)maximum=std::max(maximum,std::min(2*da[v]+c.offset_twice,2*db[v]+2*length-c.offset_twice));}
        CHECK(maximum==c.radius_twice);
    }
    cp::Graph<long long>big(2);big.add_edge(0,1,std::numeric_limits<long long>::max()-1);auto b=cp::weighted_tree_center(big);CHECK(b.offset_twice==std::numeric_limits<long long>::max()-1);
    cp::Graph<double>d(3);d.add_edge(0,1,0.5);d.add_edge(1,2,1.0);CHECK(cp::weighted_tree_center(d).offset_twice==1.5);
    cp::Graph<int>negative(2);negative.add_edge(0,1,-1);rejects([&]{cp::weighted_tree_center(negative);});
    cp::Graph<double>nan(2);nan.add_edge(0,1,std::numeric_limits<double>::quiet_NaN());rejects([&]{cp::weighted_tree_center(nan);});
}
void codes(){
    CHECK(cp::prufer_encode(cp::prufer_decode(1,{})).empty());CHECK(cp::prufer_decode(2,{}).edge_count()==1);
    for(int n=2;n<=7;++n){int total=1;for(int k=0;k<n-2;++k)total*=n;std::set<std::vector<std::pair<int,int>>> distinct;
        for(int mask=0;mask<total;++mask){int x=mask;std::vector<int>code(n-2);for(int&v:code){v=x%n;x/=n;}auto g=cp::prufer_decode(n,code);CHECK(cp::prufer_encode(g)==code);
            // 独立した「残った最小葉」の直接走査で符号化。
            std::vector<int>deg(n),naive;for(int i=0;i<n;++i)deg[i]=g[i].size();for(int k=0;k<n-2;++k){int leaf=0;while(deg[leaf]!=1)++leaf;deg[leaf]=0;for(auto e:g[leaf])if(deg[e.to]){naive.push_back(e.to);--deg[e.to];break;}}CHECK(naive==code);
            std::vector<std::pair<int,int>>es;for(auto e:g.edges())es.push_back(std::minmax(e.from,e.to));std::sort(es.begin(),es.end());CHECK(distinct.insert(es).second);
        }
    }
    rejects([]{cp::prufer_decode(0,{});});rejects([]{cp::prufer_decode(1,{0});});rejects([]{cp::prufer_decode(3,{-1});});rejects([]{cp::prufer_decode(3,{3});});rejects([]{cp::prufer_encode(cp::Graph<int>());});
    cp::Graph<int>cycle(3);cycle.add_edge(0,1);cycle.add_edge(1,2);cycle.add_edge(2,0);rejects([&]{cp::prufer_encode(cycle);});rejects([&]{cp::tree_center(cycle);});
    cp::Graph<int>forest(2);rejects([&]{cp::prufer_encode(forest);});rejects([&]{cp::tree_center(forest);});
    cp::Graph<int>multi(2);multi.add_edge(0,1);multi.add_edge(0,1);rejects([&]{cp::prufer_encode(multi);});
    cp::Graph<int>loop(1);loop.add_edge(0,0);rejects([&]{cp::prufer_encode(loop);});
    cp::Graph<int>directed(1,cp::GraphKind::Directed);rejects([&]{cp::prufer_encode(directed);});
}
void reroot(){
    std::mt19937 rng(127);
    for(int n=1;n<=55;++n){cp::Graph<int>g(n);for(int v=1;v<n;++v)if(rng()%5)g.add_edge(v,rng()%v);
        cp::HeavyLightDecomposition<int>h(g,rng()%n);
        for(int r=0;r<n;++r)for(int v=0;v<n;++v){
            if(!h.tree.same_component(r,v)){rejects([&]{cp::rerooted_subtree_intervals(h,r,v);});continue;}
            std::vector<int>p(n,-2),q{r};p[r]=-1;for(std::size_t i=0;i<q.size();++i)for(auto e:g[q[i]])if(p[e.to]==-2){p[e.to]=q[i];q.push_back(e.to);}
            std::vector<int>expected,actual;for(int u=0;u<n;++u)if(p[u]!=-2){int x=u;while(x!=-1 && x!=v)x=p[x];if(x==v)expected.push_back(u);}
            auto intervals=cp::rerooted_subtree_intervals(h,r,v);CHECK(intervals.size()<=2);int previous=-1;for(auto [l,t]:intervals){CHECK(previous<=l && l<t);previous=t;for(int i=l;i<t;++i)actual.push_back(h.vertex_at[i]);}std::sort(actual.begin(),actual.end());CHECK(actual==expected);
        }
    }
    cp::Graph<int>chain(200000);for(int i=1;i<chain.size();++i)chain.add_edge(i-1,i);CHECK(cp::tree_center(chain).radius==100000);auto code=cp::prufer_encode(chain);CHECK(cp::prufer_decode(chain.size(),code).edge_count()==chain.size()-1);
    cp::HeavyLightDecomposition<int>h(chain);CHECK(cp::rerooted_subtree_intervals(h,chain.size()-1,100)==(std::vector<std::pair<int,int>>{{0,101}}));
}
int main(){centers();codes();reroot();std::cout<<"static: all Pruefer codes through n=7, weighted/unweighted centers, all-root forest intervals, 200000-vertex chain passed\n";}
