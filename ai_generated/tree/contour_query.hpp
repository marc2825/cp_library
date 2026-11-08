#pragma once
// 説明: docs/tree/contour_query.md
#include <algorithm>
#include <cassert>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>
#include "centroid_decomposition.hpp"
#include "../range_query/fenwick_tree.hpp"
namespace cp {
// 距離は辺数。各頂点に重心祖先と、その重心で属す枝を登録する。
struct ContourIndex {
    struct Record{int centroid,distance,branch;};
    std::vector<std::vector<Record>> records;std::vector<int> all_size,branch_size;
    template<class W> explicit ContourIndex(const Graph<W>& graph):records(graph.size()),all_size(graph.size(),1){
        Graph<int> g(graph.size(),graph.kind());for(auto e:graph.edges())g.add_edge(e.from,e.to);CentroidDecomposition<int> cd(g);std::vector<int> order(g.size());std::iota(order.begin(),order.end(),0);std::sort(order.begin(),order.end(),[&](int a,int b){return cd.level[a]<cd.level[b];});std::vector<bool> removed(g.size());
        for(int center:order){removed[center]=true;records[center].push_back({center,0,-1});for(auto edge:g[center])if(!removed[edge.to]){
            int branch=static_cast<int>(branch_size.size());branch_size.push_back(1);std::vector<std::tuple<int,int,int>> stack{{edge.to,center,1}};
            while(!stack.empty()){auto [v,p,d]=stack.back();stack.pop_back();records[v].push_back({center,d,branch});all_size[center]=std::max(all_size[center],d+1);branch_size[branch]=std::max(branch_size[branch],d+1);for(auto e:g[v])if(e.to!=p && !removed[e.to])stack.emplace_back(e.to,v,d+1);}
        }}
    }
    static int shifted_bound(long long bound,int distance,int size){if(bound<=distance)return 0;return static_cast<int>(std::min<long long>(bound-distance,size));}
};
// 可換加法群。点加算・指定頂点からの距離帯[l,r)の和。
template<class T=long long> class ContourQuery {
    ContourIndex index_;std::vector<FenwickTree<T>> all_,sub_;std::vector<T> values_;
public:
    template<class W> explicit ContourQuery(const Graph<W>& g):index_(g),values_(g.size(),T(0)){for(int n:index_.all_size)all_.emplace_back(n);for(int n:index_.branch_size)sub_.emplace_back(n);}
    int size()const{return static_cast<int>(values_.size());}
    void add(int v,T x){assert(0<=v && v<size());values_[v]+=x;for(auto a:index_.records[v]){all_[a.centroid].add(a.distance,x);if(a.branch!=-1)sub_[a.branch].add(a.distance,x);}}
    void set(int v,T x){assert(0<=v && v<size());add(v,x-values_[v]);}
    T get(int v)const{assert(0<=v && v<size());return values_[v];}
    T sum(int v,long long l,long long r)const{assert(0<=v && v<size());if(l>=r)return T(0);T result=T(0);for(auto a:index_.records[v]){auto range=[&](const FenwickTree<T>& bit){return bit.sum(ContourIndex::shifted_bound(l,a.distance,bit.size()),ContourIndex::shifted_bound(r,a.distance,bit.size()));};result+=range(all_[a.centroid]);if(a.branch!=-1)result-=range(sub_[a.branch]);}return result;}
    const ContourIndex& index()const{return index_;}
};
// 距離帯へ加算・点取得。差分BITのall-subを利用する。
template<class T=long long> class ContourAddPointGet {
    ContourIndex index_;std::vector<FenwickTree<T>> all_,sub_;
public:
    template<class W> explicit ContourAddPointGet(const Graph<W>& g):index_(g){for(int n:index_.all_size)all_.emplace_back(n);for(int n:index_.branch_size)sub_.emplace_back(n);}
    void add(int v,long long l,long long r,T x){assert(0<=v && v<static_cast<int>(index_.records.size()));if(l>=r)return;for(auto a:index_.records[v]){auto apply=[&](FenwickTree<T>& bit){int left=ContourIndex::shifted_bound(l,a.distance,bit.size()),right=ContourIndex::shifted_bound(r,a.distance,bit.size());if(left<right){bit.add(left,x);if(right<bit.size())bit.add(right,T(0)-x);}};apply(all_[a.centroid]);if(a.branch!=-1)apply(sub_[a.branch]);}}
    T get(int v)const{assert(0<=v && v<static_cast<int>(index_.records.size()));T answer=T(0);for(auto a:index_.records[v]){answer+=all_[a.centroid].sum(a.distance+1);if(a.branch!=-1)answer-=sub_[a.branch].sum(a.distance+1);}return answer;}
};
} // namespace cp
