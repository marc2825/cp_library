#pragma once
// 説明: docs/tree/tree_isomorphism.md
#include <algorithm>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>
#include "centroid_decomposition.hpp"
namespace cp {
struct TreeIsomorphismResult {bool isomorphic=false;std::vector<int> mapping;};
// AHU型の完全一致。無向木の辺重みは無視。空の木同士は同型。
template<class T,class U> TreeIsomorphismResult tree_isomorphism(const Graph<T>& a,const Graph<U>& b) {
    auto topology=[](const auto& g){Graph<int> h(g.size(),g.kind());for(auto e:g.edges())h.add_edge(e.from,e.to);RootedTree<int> check(h);if(check.roots.size()>1)throw std::invalid_argument("tree isomorphism requires connected trees");return h;};
    auto x=topology(a),y=topology(b);int n=x.size();if(n!=y.size())return {};if(n==0)return {true,{}};
    auto ca=tree_centroids(x),cb=tree_centroids(y);
    for(int ra:ca)for(int rb:cb){RootedTree<int> ta(x,ra),tb(y,rb);std::map<std::vector<int>,int> dictionary;
        auto encode=[&](const Graph<int>& g,const RootedTree<int>& t){std::vector<int> code(n);for(int v:t.postorder){std::vector<int> key;for(auto e:g[v])if(t.parent[e.to]==v)key.push_back(code[e.to]);std::sort(key.begin(),key.end());code[v]=dictionary.emplace(key,static_cast<int>(dictionary.size())).first->second;}return code;};
        auto ac=encode(x,ta),bc=encode(y,tb);if(ac[ra]!=bc[rb])continue;std::vector<int> mapping(n,-1);std::vector<std::pair<int,int>> stack{{ra,rb}};
        while(!stack.empty()){auto [u,v]=stack.back();stack.pop_back();mapping[u]=v;std::vector<std::pair<int,int>> left,right;for(auto e:x[u])if(ta.parent[e.to]==u)left.emplace_back(ac[e.to],e.to);for(auto e:y[v])if(tb.parent[e.to]==v)right.emplace_back(bc[e.to],e.to);std::sort(left.begin(),left.end());std::sort(right.begin(),right.end());for(std::size_t i=0;i<left.size();++i)stack.emplace_back(left[i].second,right[i].second);}
        return {true,std::move(mapping)};
    }
    return {};
}
} // namespace cp
