#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "rooted_tree.hpp"

namespace cp {
struct TreeHash {
    int size=0; std::array<std::uint32_t,2> value{0,0};
    bool operator==(const TreeHash& other) const { return size==other.size && value==other.value; }
    bool operator!=(const TreeHash& other) const { return !(*this==other); }
    bool operator<(const TreeHash& other) const { return size!=other.size?size<other.size:value<other.value; }
};
struct RootedTreeHashResult { std::vector<TreeHash> hash; std::vector<int> parent,height,roots; };
namespace detail {
inline std::uint64_t tree_hash_random(std::uint64_t& state) {
    auto x=(state+=0x9e3779b97f4a7c15ULL); x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL; x=(x^(x>>27))*0x94d049bb133111ebULL; return x^(x>>31);
}
template<class T> Graph<int> tree_hash_unweighted(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("tree hash requires undirected forest");
    Graph<int> result(g.size()); for(const auto& e:g.edges()) result.add_edge(e.from,e.to); return result;
}
}
// ラベル・重みを無視。子の順序に依存しない二重modハッシュ。衝突はあり得る。
template<class T> RootedTreeHashResult rooted_tree_hash(const Graph<T>& g,int root=0,std::uint64_t seed=0x243f6a8885a308d3ULL) {
    auto unit=detail::tree_hash_unweighted(g); RootedTree<int> tree(unit,root); int n=g.size();
    RootedTreeHashResult result; result.hash.resize(n); result.height.assign(n,0); result.parent=tree.parent; result.roots=tree.roots;
    const std::array<std::uint64_t,2> mod{1000000007ULL,1000000009ULL};
    std::vector<std::array<std::uint64_t,2>> salt(n);
    for(auto& s:salt) for(int j=0;j<2;++j) s[j]=1+detail::tree_hash_random(seed)%(mod[j]-1);
    for(int v:tree.postorder) {
        for(const auto& e:unit[v]) if(tree.parent[e.to]==v) result.height[v]=std::max(result.height[v],result.height[e.to]+1);
        auto& h=result.hash[v]; h.size=tree.subtree_size[v]; std::array<std::uint64_t,2> value{1,1};
        for(const auto& e:unit[v]) if(tree.parent[e.to]==v) for(int j=0;j<2;++j)
            value[j]=value[j]*((salt[result.height[v]][j]+result.hash[e.to].value[j])%mod[j])%mod[j];
        for(int j=0;j<2;++j) h.value[j]=static_cast<std::uint32_t>(value[j]);
    }
    return result;
}
// 連結木全体を重心で根付ける。二重心なら小さいハッシュを採用。空木も可。
template<class T> TreeHash unrooted_tree_hash(const Graph<T>& g,std::uint64_t seed=0x243f6a8885a308d3ULL) {
    auto unit=detail::tree_hash_unweighted(g); RootedTree<int> tree(unit); int n=g.size(); if(!n) return {};
    if(tree.roots.size()!=1) throw std::invalid_argument("unrooted tree hash requires a connected tree");
    std::vector<int> centers;
    for(int v=0;v<n;++v) {
        int largest=n-tree.subtree_size[v];
        for(const auto& e:unit[v]) if(tree.parent[e.to]==v) largest=std::max(largest,tree.subtree_size[e.to]);
        if(largest<=n/2) centers.push_back(v);
    }
    TreeHash result=rooted_tree_hash(unit,centers[0],seed).hash[centers[0]];
    if(centers.size()==2) result=std::min(result,rooted_tree_hash(unit,centers[1],seed).hash[centers[1]]);
    return result;
}
} // namespace cp
