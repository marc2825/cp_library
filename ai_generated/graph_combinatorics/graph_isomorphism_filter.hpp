#pragma once
// 説明: docs/graph_combinatorics/graph_isomorphism_filter.md
#include <algorithm>
#include <map>
#include <numeric>
#include <utility>
#include <vector>
#include "../graph/graph.hpp"
namespace cp {
enum class IsomorphismStatus { different,inconclusive,isomorphic };
struct GraphIsomorphismFilterResult { IsomorphismStatus status; std::vector<int> mapping; };
// 同時1-WL精緻化。重みは無視、向き・ループ・多重辺は区別。非単色化時は未確定。
template<class T,class U> GraphIsomorphismFilterResult graph_isomorphism_filter(const Graph<T>& a,const Graph<U>& b) {
    if(a.size()!=b.size() || a.directed()!=b.directed() || a.edge_count()!=b.edge_count()) return {IsomorphismStatus::different,{}};
    int n=a.size();std::vector<std::vector<int>> out(2*n),in(2*n);
    auto fill=[&](const auto& g,int offset){for(int v=0;v<n;++v)for(auto e:g[v]){out[offset+v].push_back(offset+e.to);in[offset+e.to].push_back(offset+v);}};
    fill(a,0);fill(b,n);std::vector<int> colors(2*n);int previous_count=0;
    while(true) {
        std::map<std::vector<int>,int> dictionary;std::vector<int> next(2*n);
        for(int v=0;v<2*n;++v) {
            std::vector<int> x,y;for(int u:out[v])x.push_back(colors[u]);for(int u:in[v])y.push_back(colors[u]);std::sort(x.begin(),x.end());std::sort(y.begin(),y.end());
            std::vector<int> key{colors[v],static_cast<int>(x.size())};key.insert(key.end(),x.begin(),x.end());key.push_back(-1);key.insert(key.end(),y.begin(),y.end());
            auto it=dictionary.emplace(std::move(key),static_cast<int>(dictionary.size())).first;next[v]=it->second;
        }
        colors=std::move(next);int count=static_cast<int>(dictionary.size());std::vector<int> balance(count);
        for(int v=0;v<n;++v){++balance[colors[v]];--balance[colors[n+v]];}
        if(std::any_of(balance.begin(),balance.end(),[](int x){return x!=0;}))return {IsomorphismStatus::different,{}};
        if(count==previous_count)break;
        previous_count=count;
    }
    std::vector<int> representative(previous_count,-1),mapping(n);bool unique=true;
    for(int v=0;v<n;++v){if(representative[colors[n+v]]!=-1)unique=false;representative[colors[n+v]]=v;}
    if(!unique)return {IsomorphismStatus::inconclusive,{}};
    for(int v=0;v<n;++v)mapping[v]=representative[colors[v]];
    return {IsomorphismStatus::isomorphic,std::move(mapping)};
}
} // namespace cp
