#pragma once
// 説明: docs/graph_combinatorics/graph_isomorphism.md
#include <algorithm>
#include <functional>
#include <map>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <vector>
#include "../graph/graph.hpp"
namespace cp {
// exact yes/no。重みは無視し、向き/多重度/ループ/任意の頂点色を区別する。
// 同時色精緻化 + individualization/backtracking。最悪は指数時間。
template<class T,class U>
std::optional<std::vector<int>> graph_isomorphism(const Graph<T>& a,const Graph<U>& b,std::vector<int> colors_a={},std::vector<int> colors_b={}) {
    int n=a.size();
    if(colors_a.empty()) colors_a.resize(n);
    if(colors_b.empty()) colors_b.resize(b.size());
    if(int(colors_a.size())!=n || int(colors_b.size())!=b.size()) throw std::invalid_argument("isomorphism color size");
    if(n!=b.size() || a.directed()!=b.directed() || a.edge_count()!=b.edge_count()) return std::nullopt;
    std::vector<std::vector<int>> A(n,std::vector<int>(n)),B=A;
    for(auto e:a.edges()) { ++A[e.from][e.to]; if(!a.directed() && e.from!=e.to) ++A[e.to][e.from]; }
    for(auto e:b.edges()) { ++B[e.from][e.to]; if(!b.directed() && e.from!=e.to) ++B[e.to][e.from]; }
    std::vector<int> mapping(n,-1);
    std::function<bool(std::vector<int>,std::vector<int>)> search=[&](std::vector<int> ca,std::vector<int> cb) {
        while(true) {
            std::map<std::vector<int>,int> dictionary; std::vector<int> na(n),nb(n);
            auto refine=[&](const auto& M,const std::vector<int>& c,std::vector<int>& next) {
                for(int v=0;v<n;++v) {
                    std::map<int,std::pair<int,int>> counts;
                    for(int u=0;u<n;++u) { counts[c[u]].first+=M[v][u]; counts[c[u]].second+=M[u][v]; }
                    std::vector<int> key{c[v],M[v][v]};
                    for(auto entry:counts) { key.push_back(entry.first); key.push_back(entry.second.first); key.push_back(entry.second.second); }
                    next[v]=dictionary.emplace(std::move(key),int(dictionary.size())).first->second;
                }
            };
            refine(A,ca,na); refine(B,cb,nb);
            auto sa=na,sb=nb; std::sort(sa.begin(),sa.end()); std::sort(sb.begin(),sb.end()); if(sa!=sb) return false;
            int old_count=0; { auto old=ca; std::sort(old.begin(),old.end()); old_count=int(std::unique(old.begin(),old.end())-old.begin()); }
            ca=std::move(na); cb=std::move(nb); if(int(dictionary.size())==old_count) break;
        }
        std::map<int,std::vector<int>> classes_a,classes_b;
        for(int v=0;v<n;++v) { classes_a[ca[v]].push_back(v); classes_b[cb[v]].push_back(v); }
        int chosen=-1,size=n+1;
        for(const auto& entry:classes_a) if(entry.second.size()>1 && int(entry.second.size())<size) { chosen=entry.first; size=int(entry.second.size()); }
        if(chosen<0) {
            for(const auto& entry:classes_a) mapping[entry.second[0]]=classes_b[entry.first][0];
            for(int u=0;u<n;++u) for(int v=0;v<n;++v) if(A[u][v]!=B[mapping[u]][mapping[v]]) return false;
            return true;
        }
        int v=classes_a[chosen][0],fresh=int(classes_a.size()); ca[v]=fresh;
        for(int u:classes_b[chosen]) { auto next=cb; next[u]=fresh; if(search(ca,std::move(next))) return true; }
        return false;
    };
    if(search(std::move(colors_a),std::move(colors_b))) return mapping;
    return std::nullopt;
}
} // namespace cp
