#pragma once
// 説明: docs/graph_combinatorics/euler_circuit_count.md
#include <stdexcept>
#include <vector>
#include "../graph/graph.hpp"
#include "determinant_mod.hpp"
namespace cp {
// 始点 root の辺ID列を数える。first_edge>=0 は先頭の辺を固定。素数法。
template<int Mod=998244353,class T>
int count_euler_circuits(const Graph<T>& g,int root,int first_edge=-1) {
    if(!g.directed()) throw std::invalid_argument("directed Euler count required");
    int n=g.size(); if(root<0 || root>=n) throw std::out_of_range("Euler root");
    if(first_edge < -1 || first_edge>=g.edge_count() || (first_edge>=0 && g.edges()[first_edge].from!=root)) throw std::invalid_argument("invalid first edge");
    if(!g.edge_count()) return 1;
    std::vector<int> in(n),out(n),index(n,-1);
    for(auto e:g.edges()) { ++out[e.from]; ++in[e.to]; }
    if(!out[root]) return 0;
    int active=0;
    for(int v=0;v<n;++v) { if(in[v]!=out[v]) return 0; if(out[v] && v!=root) index[v]=active++; }
    std::vector<std::vector<long long>> minor(active,std::vector<long long>(active));
    for(auto e:g.edges()) if(e.from!=e.to && e.from!=root) {
        int a=index[e.from]; minor[a][a]=(minor[a][a]+1)%Mod;
        if(e.to!=root) { int b=index[e.to]; minor[a][b]=(minor[a][b]+Mod-1)%Mod; }
    }
    long long answer=determinant_mod<Mod>(std::move(minor));
    for(int d:out) for(int k=2;k<d;++k) answer=answer*k%Mod;
    if(first_edge<0) answer=answer*out[root]%Mod;
    return int(answer);
}
} // namespace cp
