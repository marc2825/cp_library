#include "headers.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#define CHECK(x) do {if(!(x)){std::cerr<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
using namespace cp;
template<class E,class F>void throws(F f){bool caught=false;try{f();}catch(const E&){caught=true;}CHECK(caught);}
int main(){
    throws<std::invalid_argument>([]{SegmentTreeGraph<> x(-1);});
    throws<std::out_of_range>([]{SegmentTreeGraph<> x(3);x.add_point_to_range(0,-1,2,1);});
    throws<std::out_of_range>([]{bipartite_vertex_cover(1,1,{{0,1}});});
    Graph<int> loop(1);loop.add_edge(0,0);
    throws<std::invalid_argument>([&]{minimum_edge_cover(loop);});
    throws<std::invalid_argument>([&]{vizing_edge_coloring(loop);});
    Graph<int> parallel(2);parallel.add_edge(0,1);parallel.add_edge(1,0);
    throws<std::invalid_argument>([&]{vizing_edge_coloring(parallel);});
    CHECK(minimum_edge_cover(parallel)->size()==1);CHECK(st_numbering(parallel,0,1));
    Graph<int> directed(2,GraphKind::Directed);directed.add_edge(0,1);directed.add_edge(1,0);
    throws<std::invalid_argument>([&]{gomory_hu_tree(directed);});
    throws<std::invalid_argument>([&]{dag_maximum_antichain(directed);});
    throws<std::invalid_argument>([&]{minimum_mean_cycle(parallel);});
    throws<std::out_of_range>([&]{find_negative_cycle(directed,2);});
    throws<std::invalid_argument>([]{DifferenceConstraints<> x(-1);});
    throws<std::out_of_range>([]{DifferenceConstraints<> x(1);x.add_constraint(0,1,0);});
    throws<std::out_of_range>([]{IncrementalMSF<> x(1);x.add_edge(0,1,0);});
    throws<std::invalid_argument>([]{boruvka_msf<int>(2,[](const auto&){return std::vector<std::optional<BoruvkaEdge<int>>>{};});});
    throws<std::invalid_argument>([]{boruvka_msf<int>(2,[](const auto&){return std::vector<std::optional<BoruvkaEdge<int>>>{{BoruvkaEdge<int>{0,0,1}},std::nullopt};});});
    Graph<long long> big(2);big.add_edge(0,1,std::numeric_limits<long long>::max());CHECK(gomory_hu_tree(big).edges()[0].cost==std::numeric_limits<long long>::max());
    big.add_edge(0,1,1);throws<std::overflow_error>([&]{gomory_hu_tree(big);});
    Graph<int> negative(2);negative.add_edge(0,1,-1);
    throws<std::invalid_argument>([&]{gomory_hu_tree(negative);});
    throws<std::invalid_argument>([&]{min_cost_edge_disjoint_paths(negative,0,1,1);});
    throws<std::invalid_argument>([&]{min_cost_edge_disjoint_paths(parallel,0,0,0);});
    throws<std::invalid_argument>([&]{st_numbering(parallel,0,0);});
    Graph<long long> expensive(3,GraphKind::Directed);expensive.add_edge(0,1,std::numeric_limits<long long>::max());expensive.add_edge(1,2,1);
    throws<std::overflow_error>([&]{min_cost_edge_disjoint_paths(expensive,0,2,1);});
    Graph<int> empty;CHECK(minimum_edge_cover(empty)->empty());CHECK(gomory_hu_tree(empty).size()==0);CHECK(!find_negative_cycle(empty));
    CHECK(DifferenceConstraints<>(0).solve().feasible());CHECK(vizing_edge_coloring(empty).color_count==0);
    Graph<int> no_cycle(0,GraphKind::Directed);CHECK(!minimum_mean_cycle(no_cycle));CHECK(dag_maximum_antichain(no_cycle).antichain.empty());
    std::mt19937 random(417);
    for(int count=1;count<=65;++count) {
        Graph<int> dense(count);
        for(int u=0;u<count;++u)for(int v=u+1;v<count;++v)if(count%2 || random()%3)dense.add_edge(u,v);
        auto coloring=vizing_edge_coloring(dense);CHECK(coloring.color_count<=coloring.max_degree+1);
        for(int v=0;v<count;++v){std::set<int> used;for(auto e:dense[v])CHECK(used.insert(coloring.color[e.idx]).second);}
    }
    for(int trial=0;trial<160;++trial) {
        int count=2+int(random()%6);Graph<int> g(count);
        for(int i=0;i<20;++i)g.add_edge(int(random()%count),int(random()%count),int(random()%8));
        auto tree=gomory_hu_tree(g);
        for(int s=0;s<count;++s)for(int t=s+1;t<count;++t) {
            int best=10000;
            for(int side=0;side<(1<<count);++side)if((side>>s&1) && !(side>>t&1)) {
                int cost=0;for(auto e:g.edges())if(((side>>e.from)^(side>>e.to))&1)cost+=e.cost;best=std::min(best,cost);
            }
            std::vector<int> q{s},d(count,-1);d[s]=10000;
            for(std::size_t i=0;i<q.size();++i)for(auto e:tree[q[i]])if(d[e.to]<0){d[e.to]=std::min(d[e.from],e.cost);q.push_back(e.to);}
            CHECK(d[t]==best);
        }
    }
    // Deep inputs test iterative traversal and LCT growth/reuse, including NDEBUG.
    const int n=50000;
    Graph<int> chain(n);for(int i=1;i<n;++i)chain.add_edge(i-1,i);
    auto st=st_numbering(chain,0,n-1);CHECK(st && int(st->order.size())==n);
    chain.add_edge(n-1,0);CHECK(unicyclic_decomposition(chain)->cycle_edge_ids.size()==std::size_t(n));
    SegmentTreeGraph<> range(n);range.add_range_to_range(0,n,0,n,1);CHECK(range.vertex_count()<4*n);
    IncrementalMSF<> forest(n);for(int i=1;i<n;++i)forest.add_edge(i-1,i,2);for(int i=1;i<n;++i)forest.add_edge(0,i,1);CHECK(forest.cost()==n-1 && forest.components()==1);
    std::vector<std::uint64_t> values(4096);for(int i=0;i<int(values.size());++i)values[i]=std::uint64_t(i);
    CHECK(xor_minimum_spanning_tree(values).cost==24576); // Sum over 12 binary levels: 12 * 2^11.
    std::cout<<"invalid inputs, integer limits, 50000-vertex traversal/LCT and 4096 XOR vertices passed\n";
}
