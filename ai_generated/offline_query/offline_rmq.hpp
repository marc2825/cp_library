#pragma once
#include <cassert>
#include <functional>
#include <utility>
#include <vector>

namespace cp {
// 単調スタック＋次の小さい位置へのリンク。最左argmin、空区間は-1。
template<class T,class Compare=std::less<T>> std::vector<int> offline_rmq(
    const std::vector<T>& a,const std::vector<std::pair<int,int>>& queries,Compare compare=Compare{}) {
    int n=int(a.size()); std::vector<std::vector<std::pair<int,int>>> ends(n+1);
    std::vector<int> result(queries.size(),-1),parent(n),stack;
    for(int id=0;id<int(queries.size());++id) {
        auto [l,r]=queries[id]; assert(0<=l && l<=r && r<=n);
        if(l<r) ends[r].emplace_back(l,id);
    }
    auto find=[&](int x) {
        int r=x; while(parent[r]!=r) r=parent[r];
        while(parent[x]!=x) { int y=parent[x]; parent[x]=r; x=y; }
        return r;
    };
    for(int i=0;i<n;++i) {
        parent[i]=i;
        while(!stack.empty() && compare(a[i],a[stack.back()])) { parent[stack.back()]=i; stack.pop_back(); }
        stack.push_back(i);
        for(auto [l,id]:ends[i+1]) result[id]=find(l);
    }
    return result;
}
} // namespace cp
