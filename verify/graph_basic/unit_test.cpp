#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <vector>
#include "graph_basic/all.hpp"
#include "graph/all.hpp"
#include "tree/all.hpp"

#define CHECK(x) do { if (!(x)) { std::cerr << "line " << __LINE__ << ": " #x "\n"; std::abort(); } } while (false)
using G = cp::Graph<long long>;
constexpr long long inf = 1000000000000LL;

std::vector<long long> bounded_paths(const G& g, int s, int rounds, bool unit = false) {
    std::vector<long long> d(g.size(), inf); d[s] = 0;
    for (int k = 0; k < rounds; ++k) {
        auto next = d;
        for (int u = 0; u < g.size(); ++u) if (d[u] < inf)
            for (auto e : g[u]) next[e.to] = std::min(next[e.to], d[u] + (unit ? 1 : e.cost));
        d = std::move(next);
    }
    return d;
}
template <class T> void check_path(const G& g, const cp::ShortestPathResult<T>& r, int t, bool unit = false) {
    auto v = r.vertex_path(t), e = r.edge_path(t);
    if (!r.has_finite_distance(t)) { CHECK(v.empty() && e.empty()); return; }
    CHECK(v.front() == r.source && v.back() == t && v.size() == e.size() + 1);
    CHECK(std::set<int>(v.begin(), v.end()).size() == v.size());
    long long sum = 0;
    for (int i = 0; i < int(e.size()); ++i) {
        auto edge = g.edges()[e[i]];
        CHECK((edge.from == v[i] && edge.to == v[i+1]) || (!g.directed() && edge.to == v[i] && edge.from == v[i+1]));
        sum += unit ? 1 : edge.cost;
    }
    CHECK(sum == r.dist[t]);
}
void check_dfs(const G& g, int first) {
    auto got = cp::dfs_forest(g, first);
    cp::DFSForestResult expected;
    int n = g.size();
    expected.parent.assign(n,-1); expected.parent_edge.assign(n,-1);
    expected.depth.assign(n,0); expected.component.assign(n,-1);
    expected.tin.assign(n,-1); expected.tout.assign(n,-1);
    std::function<void(int,int)> visit = [&](int v,int component) {
        expected.component[v] = component; expected.tin[v] = int(expected.preorder.size());
        expected.preorder.push_back(v);
        for (auto e : g[v]) if (expected.component[e.to] < 0) {
            expected.parent[e.to] = v; expected.parent_edge[e.to] = e.idx;
            expected.depth[e.to] = expected.depth[v] + 1; visit(e.to,component);
        }
        expected.tout[v] = int(expected.preorder.size()); expected.postorder.push_back(v);
    };
    auto root = [&](int v) { if (expected.component[v] < 0) { expected.roots.push_back(v); visit(v,int(expected.roots.size())-1); } };
    if (first >= 0) root(first);
    for (int v = 0; v < n; ++v) root(v);
    CHECK(got.parent == expected.parent && got.parent_edge == expected.parent_edge && got.depth == expected.depth);
    CHECK(got.roots == expected.roots && got.component == expected.component);
    CHECK(got.preorder == expected.preorder && got.postorder == expected.postorder && got.tin == expected.tin && got.tout == expected.tout);
}
void check_mst(const G& g) {
    auto a = cp::kruskal(g), b = cp::prim(g);
    auto components = [&](const std::vector<int>& ids) {
        std::vector<std::vector<int>> adj(g.size());
        for (int id : ids) { auto e=g.edges()[id]; adj[e.from].push_back(e.to); adj[e.to].push_back(e.from); }
        std::vector<int> label(g.size(),-1); int c=0;
        for (int s=0;s<g.size();++s) if (label[s]<0) {
            std::vector<int> q{s}; label[s]=c++;
            for (int i=0;i<int(q.size());++i) for (int v:adj[q[i]]) if(label[v]<0){label[v]=label[s];q.push_back(v);}
        }
        return std::make_pair(c,label);
    };
    std::vector<int> all(g.edge_count()); std::iota(all.begin(),all.end(),0);
    auto [count, labels] = components(all);
    long long best=inf;
    for (int mask=0;mask<(1<<g.edge_count());++mask) {
        std::vector<int> ids; long long cost=0;
        for (int i=0;i<g.edge_count();++i) if(mask>>i&1){ids.push_back(i);cost+=g.edges()[i].cost;}
        if(int(ids.size())!=g.size()-count)continue;
        if(components(ids).second==labels)best=std::min(best,cost);
    }
    CHECK(a.cost==best && b.cost==best && a.component_count==count && b.component_count==count);
    for (const auto& result:{a,b}) {
        std::vector<int> ids;
        for(auto e:result.edges)ids.push_back(e.idx);
        CHECK(int(ids.size())==g.size()-count && components(ids).second==labels);
    }
}
void random_graphs() {
    std::mt19937 rng(37001);
    for (int trial=0;trial<600;++trial) {
        int n=rng()%8;
        G g(n,trial%2?cp::GraphKind::Directed:cp::GraphKind::Undirected);
        bool binary=trial%3==0, nonnegative=trial%3!=2;
        if(n) for(int i=0,m=rng()%10;i<m;++i)g.add_edge(rng()%n,rng()%n,binary?int(rng()%2):nonnegative?int(rng()%8):int(rng()%13)-6);
        auto all=cp::floyd_warshall(g);
        check_dfs(g,-1); if(n)check_dfs(g,int(rng()%n));
        if(!g.directed())check_mst(g);
        for(int s=0;s<n;++s){
            auto expected=bounded_paths(g,s,n-1),more=bounded_paths(g,s,n);
            std::vector<bool> bad(n,false);
            std::vector<int> q;
            for(int v=0;v<n;++v)if(more[v]<expected[v]){bad[v]=true;q.push_back(v);}
            for(int i=0;i<int(q.size());++i)for(auto e:g[q[i]])if(!bad[e.to]){bad[e.to]=true;q.push_back(e.to);}
            auto bellman=cp::bellman_ford(g,s);
            auto bfs=cp::bfs(g,s),dfs=cp::depth_first_search(g,s);
            auto unweighted=bounded_paths(g,s,n-1,true);
            for(int t=0;t<n;++t){
                CHECK(bellman.negative_cycle[t]==bad[t] && all[s].negative_cycle[t]==bad[t]);
                CHECK(bellman.reachable(t)==(expected[t]<inf) && all[s].reachable(t)==(expected[t]<inf));
                if(expected[t]<inf&&!bad[t])CHECK(bellman.dist[t]==expected[t]&&all[s].dist[t]==expected[t]);
                CHECK(bfs.reachable(t)==(expected[t]<inf)&&dfs.reachable(t)==(expected[t]<inf));
                if(bfs.reachable(t))CHECK(bfs.dist[t]==unweighted[t]);
                check_path(g,bellman,t);check_path(g,all[s],t);check_path(g,bfs,t,true);check_path(g,dfs,t,true);
            }
            if(nonnegative){auto d=cp::dijkstra(g,s);for(int t=0;t<n;++t){CHECK(d.reachable(t)==(expected[t]<inf));if(d.reachable(t))CHECK(d.dist[t]==expected[t]);check_path(g,d,t);}}
            if(binary){auto d=cp::zero_one_bfs(g,s);for(int t=0;t<n;++t){CHECK(d.reachable(t)==(expected[t]<inf));if(d.reachable(t))CHECK(d.dist[t]==expected[t]);check_path(g,d,t);}}
        }
        if(g.directed()){
            std::vector<int> permutation(n);std::iota(permutation.begin(),permutation.end(),0);bool dag=false;
            do{std::vector<int> pos(n);for(int i=0;i<n;++i)pos[permutation[i]]=i;bool valid=true;for(auto e:g.edges())valid&=pos[e.from]<pos[e.to];dag|=valid;}while(std::next_permutation(permutation.begin(),permutation.end()));
            auto order=cp::topological_sort(g);CHECK(order.has_value()==dag);
            CHECK(cp::find_cycle(g).has_value()==!dag);
        }
    }
    std::cout<<"600 graphs: bounded-walk oracle, negative influence, every restored path, recursive DFS, permutation DAG and subset MSF passed\n";
}
void large() {
    constexpr int n=200000;
    G g(n,cp::GraphKind::Directed);
    for(int i=0;i<n-1;++i)g.add_edge(i,i+1,1);
    auto dfs=cp::dfs_forest(g);CHECK(dfs.preorder.front()==0&&dfs.postorder.front()==n-1&&dfs.tout[0]==n);
    CHECK(cp::bfs(g).dist.back()==n-1&&cp::dijkstra(g).dist.back()==n-1&&cp::zero_one_bfs(g).dist.back()==n-1);
    CHECK(cp::topological_sort(g)->size()==n&&!cp::find_cycle(g));
    auto path=cp::dijkstra(g).vertex_path(n-1);CHECK(path.size()==n);
    G forest(n);for(int i=0;i<n-1;++i)forest.add_edge(i,i+1,1);
    CHECK(cp::kruskal(forest).cost==n-1&&cp::prim(forest).cost==n-1);
    G sparse(n,cp::GraphKind::Directed);sparse.add_edge(0,0,-1);
    auto bellman=cp::bellman_ford(sparse);CHECK(bellman.negative_cycle[0]&&!bellman.reachable(n-1));
    std::cout<<"200000-vertex chain: iterative DFS/cycle, BFS/0-1/Dijkstra, DAG, MSF and restoration passed\n";
}
int main(){random_graphs();large();}
