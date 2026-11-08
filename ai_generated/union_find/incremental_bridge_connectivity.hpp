#pragma once
// 説明: docs/union_find/incremental_bridge_connectivity.md
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

namespace cp {
// 辺追加のみ。連結成分と二重辺連結成分、現在の橋数。
class IncrementalBridgeConnectivity {
    std::vector<int> block_,component_,size_,parent_;
    std::vector<std::uint64_t> visited_; std::uint64_t iteration_=0; int bridges_=0;
    int block(int v) {
        if(v<0) return -1;
        int r=v; while(block_[r]!=r) r=block_[r];
        while(v!=r) { int next=block_[v]; block_[v]=r; v=next; } return r;
    }
    int component(int v) {
        v=block(v); int r=v;
        while(component_[r]!=r) r=block(component_[r]);
        while(v!=r) { int next=block(component_[v]); component_[v]=r; v=next; } return r;
    }
    void reroot(int v) {
        v=block(v); int root=v,child=-1;
        while(v!=-1) { int next=block(parent_[v]); parent_[v]=child; component_[v]=root; child=v; v=next; }
        size_[root]=size_[child];
    }
    void close_cycle(int a,int b) {
        if(++iteration_==0) { std::fill(visited_.begin(),visited_.end(),0); ++iteration_; }
        std::vector<int> pa,pb; int ancestor=-1;
        while(ancestor<0) {
            for(int side=0;side<2;++side) {
                int& v=side?b:a; auto& path=side?pb:pa;
                if(v<0) continue;
                v=block(v); path.push_back(v);
                if(visited_[v]==iteration_) { ancestor=v; break; }
                visited_[v]=iteration_; v=parent_[v];
            }
        }
        for(const auto& path:{pa,pb}) for(int v:path) { block_[v]=ancestor; if(v==ancestor) break; --bridges_; }
    }
    void check(int v) const { assert(0<=v && v<int(block_.size())); (void)v; }
public:
    explicit IncrementalBridgeConnectivity(int n) : block_(n),component_(n),size_(n,1),parent_(n,-1),visited_(n) {
        for(int i=0;i<n;++i) block_[i]=component_[i]=i;
    }
    int bridge_count() const { return bridges_; }
    bool same(int a,int b) { check(a); check(b); return component(a)==component(b); }
    bool two_edge_connected(int a,int b) { check(a); check(b); return block(a)==block(b); }
    void add_edge(int a,int b) {
        check(a); check(b); a=block(a); b=block(b); if(a==b) return;
        int ca=component(a),cb=component(b);
        if(ca==cb) { close_cycle(a,b); return; }
        ++bridges_;
        if(size_[ca]>size_[cb]) { std::swap(a,b); std::swap(ca,cb); }
        reroot(a); parent_[a]=b; component_[a]=b; size_[cb]+=size_[a];
    }
};
} // namespace cp
