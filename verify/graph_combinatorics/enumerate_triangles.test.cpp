#define PROBLEM "https://judge.yosupo.jp/problem/enumerate_triangles"
#include <iostream>
#include <vector>
#include "graph_combinatorics/triangle_enumeration.hpp"
int main() {
    int n,m; std::cin>>n>>m; cp::Graph<> g(n);
    const long long mod=998244353; std::vector<long long> x(n); for(auto& value:x) std::cin>>value;
    for(int i=0;i<m;++i) { int u,v; std::cin>>u>>v; g.add_edge(u,v); }
    long long answer=0; cp::enumerate_triangles(g,[&](int u,int v,int w) { answer=(answer+x[u]*x[v]%mod*x[w])%mod; return true; });
    std::cout<<answer<<'\n';
}
