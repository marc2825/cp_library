#pragma once
// 説明: docs/graph_connectivity/cograph_canonical_form.md
#include <algorithm>
#include <functional>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include "graph.hpp"
namespace cp {
struct CographCanonicalForm {
    enum class Kind { leaf,disjoint_union,join };
    struct Node { Kind kind; int vertex=-1; std::vector<int> children; };
    std::string code; std::vector<int> vertex_order; std::vector<Node> cotree; int root=-1;
};
// 単純無向 cograph の完全な標準形。非 cograph は nullopt。色は任意の int。
template<class T> std::optional<CographCanonicalForm> cograph_canonical_form(const Graph<T>& g,std::vector<int> colors={}) {
    if(g.directed()) throw std::invalid_argument("undirected cograph required");
    int n=g.size(); if(colors.empty()) colors.resize(n); if(int(colors.size())!=n) throw std::invalid_argument("color size");
    std::vector<std::vector<bool>> a(n,std::vector<bool>(n));
    for(auto e:g.edges()) { if(e.from==e.to || a[e.from][e.to]) throw std::invalid_argument("simple cograph required"); a[e.from][e.to]=a[e.to][e.from]=true; }
    CographCanonicalForm r; std::vector<std::string> codes;
    std::function<int(const std::vector<int>&)> solve=[&](const std::vector<int>& vertices)->int {
        if(vertices.size()==1) {
            int id=int(r.cotree.size()),v=vertices[0]; r.cotree.push_back({CographCanonicalForm::Kind::leaf,v,{}}); codes.push_back("L"+std::to_string(colors[v])+";"); return id;
        }
        for(int complement=0;complement<2;++complement) {
            std::vector<bool> seen(n); std::vector<std::vector<int>> components;
            for(int start:vertices) if(!seen[start]) {
                components.push_back({start}); seen[start]=true; auto& c=components.back();
                for(std::size_t i=0;i<c.size();++i) for(int u:vertices) if(!seen[u] && (a[c[i]][u]!=bool(complement))) { seen[u]=true; c.push_back(u); }
            }
            if(components.size()>1) {
                std::vector<int> children;
                for(const auto& c:components) { int child=solve(c); if(child<0) return -1; children.push_back(child); }
                std::sort(children.begin(),children.end(),[&](int x,int y){return codes[x]<codes[y];});
                std::string code=complement?"J(":"U("; for(int id:children) code+=codes[id]; code+=')';
                int id=int(r.cotree.size()); r.cotree.push_back({complement?CographCanonicalForm::Kind::join:CographCanonicalForm::Kind::disjoint_union,-1,std::move(children)}); codes.push_back(std::move(code)); return id;
            }
        }
        return -1;
    };
    if(!n) { r.code="E"; return r; }
    std::vector<int> vertices(n); std::iota(vertices.begin(),vertices.end(),0); r.root=solve(vertices); if(r.root<0) return std::nullopt;
    r.code=codes[r.root]; std::vector<int> stack{r.root};
    while(!stack.empty()) { int id=stack.back(); stack.pop_back(); auto& node=r.cotree[id]; if(node.vertex>=0) r.vertex_order.push_back(node.vertex); else for(auto it=node.children.rbegin();it!=node.children.rend();++it) stack.push_back(*it); }
    return r;
}
} // namespace cp
