#pragma once
#include <cassert>
#include <cstdint>
#include <optional>
#include <queue>
#include <vector>
#include "../tree/binary_lifting.hpp"

namespace cp {
// 各頂点の出次数がちょうど1。閉路＋流入木を分解。-1終端は許容しない。
class FunctionalGraph {
    BinaryLifting lifting_;
public:
    std::vector<int> next,component,depth,entry,position;
    std::vector<std::vector<int>> cycles;
    explicit FunctionalGraph(const std::vector<int>& successors):lifting_(successors),next(successors) {
        int n=int(next.size()); std::vector<int> degree(n),order;
        for(int v:next) { assert(0<=v && v<n); ++degree[v]; }
        std::queue<int> queue; for(int v=0;v<n;++v) if(!degree[v]) queue.push(v);
        while(!queue.empty()) { int v=queue.front(); queue.pop(); order.push_back(v); if(--degree[next[v]]==0) queue.push(next[v]); }
        component.assign(n,-1); depth.assign(n,0); entry.assign(n,-1); position.assign(n,-1);
        for(int v=0;v<n;++v) if(degree[v] && component[v]==-1) {
            int id=int(cycles.size()); cycles.push_back({}); int u=v;
            do { component[u]=id; entry[u]=u; position[u]=int(cycles.back().size()); cycles.back().push_back(u); u=next[u]; } while(u!=v);
        }
        for(auto it=order.rbegin();it!=order.rend();++it) {
            int v=*it,u=next[v]; component[v]=component[u]; depth[v]=depth[u]+1; entry[v]=entry[u]; position[v]=position[u];
        }
    }
    int size() const { return int(next.size()); }
    int component_count() const { return int(cycles.size()); }
    int jump(int v,std::uint64_t steps) const { return lifting_.jump(v,steps); }
    bool on_cycle(int v) const { assert(0<=v && v<size()); return depth[v]==0; }
    std::optional<std::uint64_t> min_steps(int u,int v) const {
        assert(0<=u && u<size() && 0<=v && v<size());
        if(component[u]!=component[v]) return std::nullopt;
        if(depth[v]) {
            if(depth[u]<depth[v]) return std::nullopt;
            auto d=static_cast<std::uint64_t>(depth[u]-depth[v]);
            if(jump(u,d)!=v) return std::nullopt;
            return d;
        }
        int length=int(cycles[component[u]].size());
        int offset=int((static_cast<long long>(position[v])-position[u]+length)%length);
        return static_cast<std::uint64_t>(depth[u])+offset;
    }
};
} // namespace cp
