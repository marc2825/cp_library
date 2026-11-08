#pragma once
// 説明: docs/graph_flow/rank_maximal_matching.md
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <vector>
#include "../basic_utility/big_integer.hpp"
namespace cp {
struct RankedBipartiteEdge { int left,right,rank; };
struct RankMaximalMatchingResult {
    std::vector<int> left_match,right_match,edge_ids,ranks,profile;
};
// rank が小さいほど優先。辞書式目的を多倍長整数で厳密に符号化する。
inline RankMaximalMatchingResult rank_maximal_matching(int left,int right,const std::vector<RankedBipartiteEdge>& edges) {
    if(left<0 || right<0) throw std::invalid_argument("negative matching size");
    RankMaximalMatchingResult r; r.left_match.assign(left,-1); r.right_match.assign(right,-1);
    for(auto e:edges) {
        if(e.left<0 || e.left>=left || e.right<0 || e.right>=right || e.rank<=0) throw std::invalid_argument("invalid ranked edge");
        r.ranks.push_back(e.rank);
    }
    std::sort(r.ranks.begin(),r.ranks.end()); r.ranks.erase(std::unique(r.ranks.begin(),r.ranks.end()),r.ranks.end()); r.profile.resize(r.ranks.size());
    std::vector<BigInt> weights(r.ranks.size(),BigInt(1));
    for(int i=int(weights.size())-2;i>=0;--i) weights[i]=weights[i+1]*BigInt(std::min(left,right)+1);
    std::vector<std::vector<int>> ids(left,std::vector<int>(right,-1));
    for(int i=0;i<int(edges.size());++i) { auto e=edges[i]; int& id=ids[e.left][e.right]; if(id<0 || e.rank<edges[id].rank) id=i; }
    int columns=right+left;
    std::vector<BigInt> u(left+1),v(columns+1); std::vector<int> p(columns+1),way(columns+1);
    auto cost=[&](int a,int b)->std::optional<BigInt> {
        if(b>=right) return BigInt(0);
        int id=ids[a][b]; if(id<0) return std::nullopt;
        return -weights[std::lower_bound(r.ranks.begin(),r.ranks.end(),edges[id].rank)-r.ranks.begin()];
    };
    for(int row=1;row<=left;++row) {
        p[0]=row; int j0=0; std::vector<std::optional<BigInt>> dist(columns+1); std::vector<bool> used(columns+1);
        do {
            used[j0]=true; int i0=p[j0],j1=-1; std::optional<BigInt> delta;
            for(int j=1;j<=columns;++j) if(!used[j]) {
                auto c=cost(i0-1,j-1);
                if(c) { BigInt d=*c-u[i0]-v[j]; if(!dist[j] || d<*dist[j]) { dist[j]=d; way[j]=j0; } }
                if(dist[j] && (!delta || *dist[j]<*delta)) { delta=dist[j]; j1=j; }
            }
            if(!delta) throw std::logic_error("missing dummy assignment");
            for(int j=0;j<=columns;++j) {
                if(used[j]) { u[p[j]]+=*delta; v[j]-=*delta; }
                else if(dist[j]) *dist[j]-=*delta;
            }
            j0=j1;
        } while(p[j0]);
        do { int j1=way[j0]; p[j0]=p[j1]; j0=j1; } while(j0);
    }
    for(int j=1;j<=right;++j) if(p[j]) {
        int a=p[j]-1,b=j-1,id=ids[a][b]; r.left_match[a]=b; r.right_match[b]=a; r.edge_ids.push_back(id);
        ++r.profile[std::lower_bound(r.ranks.begin(),r.ranks.end(),edges[id].rank)-r.ranks.begin()];
    }
    return r;
}
} // namespace cp
