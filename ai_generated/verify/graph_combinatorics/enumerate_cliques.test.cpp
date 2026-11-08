#define PROBLEM "https://judge.yosupo.jp/problem/enumerate_cliques"
#include <iostream>
#include <vector>
#include "graph_combinatorics/clique_enumeration.hpp"
int main() {
    int n,m; std::cin>>n>>m; cp::Graph<> g(n);
    const long long mod=998244353; std::vector<long long> x(n); for(auto& value:x) std::cin>>value;
    for(int i=0;i<m;++i) { int u,v; std::cin>>u>>v; g.add_edge(u,v); }
    long long answer=0; cp::enumerate_cliques(g,[&](const auto& vertices) { long long product=1; for(int v:vertices) product=product*x[v]%mod; answer=(answer+product)%mod; return true; });
    std::cout<<answer<<'\n';
}
