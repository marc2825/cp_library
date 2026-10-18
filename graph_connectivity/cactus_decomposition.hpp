#pragma once
// 説明: docs/graph_connectivity/cactus_decomposition.md
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>
#include "low_link.hpp"

namespace cp {
struct CactusBlock { std::vector<int> vertices,edge_ids,cycle,cycle_edge_ids; };
struct CactusDecompositionResult {
    bool is_cactus=true;
    std::vector<CactusBlock> blocks;
    Graph<int> block_cut_forest;
};
// 無向多重グラフ。loopは長さ1、平行2辺は長さ2のcycleとして扱う。
template<class T> CactusDecompositionResult cactus_decomposition(const Graph<T>& g) {
    if(g.directed()) throw std::invalid_argument("cactus requires undirected graph");
    int n=g.size(); auto low=low_link(g); CactusDecompositionResult answer;
    std::vector<int> edge_stack,seen(n,-1); std::vector<std::pair<int,int>> incidence(n,{-1,0});
    auto add_block=[&](std::vector<int> ids) {
        CactusBlock block; block.edge_ids=std::move(ids); int stamp=int(answer.blocks.size());
        std::vector<std::vector<std::pair<int,int>>> adjacent;
        for(int id:block.edge_ids) {
            const auto& e=g.edges()[id];
            for(int v:{e.from,e.to}) {
                if(seen[v]!=stamp) { seen[v]=stamp; incidence[v]={stamp,int(block.vertices.size())}; block.vertices.push_back(v); adjacent.push_back({}); }
            }
            adjacent[incidence[e.from].second].emplace_back(e.to,id); adjacent[incidence[e.to].second].emplace_back(e.from,id);
        }
        bool cycle=block.edge_ids.size()==block.vertices.size();
        for(const auto& row:adjacent) cycle=cycle && row.size()==2;
        if(cycle) {
            int start=*std::min_element(block.vertices.begin(),block.vertices.end()),v=start,previous=-1;
            block.cycle.push_back(start);
            for(std::size_t i=0;i<block.edge_ids.size();++i) {
                const auto& row=adjacent[incidence[v].second]; auto e=row[0].second!=previous?row[0]:row[1];
                block.cycle_edge_ids.push_back(e.second); v=e.first; block.cycle.push_back(v); previous=e.second;
            }
        } else if(block.edge_ids.size()!=1) answer.is_cactus=false;
        answer.blocks.push_back(std::move(block));
    };
    for(const auto& e:g.edges()) if(e.from==e.to) add_block({e.idx});
    for(int start=0;start<n;++start) if(low.parent[start]<0) {
        std::vector<std::pair<int,std::size_t>> stack{{start,0}};
        while(!stack.empty()) {
            int v=stack.back().first; auto& next=stack.back().second;
            if(next<g[v].size()) {
                auto e=g[v][next++]; if(e.from==e.to || e.idx==low.parent_edge[v]) continue;
                if(low.parent[e.to]==v && low.parent_edge[e.to]==e.idx) { edge_stack.push_back(e.idx); stack.emplace_back(e.to,0); }
                else if(low.ord[e.to]<low.ord[v]) edge_stack.push_back(e.idx);
            } else {
                stack.pop_back(); int p=low.parent[v]; if(p<0 || low.low[v]<low.ord[p]) continue;
                std::vector<int> ids;
                do { ids.push_back(edge_stack.back()); edge_stack.pop_back(); } while(ids.back()!=low.parent_edge[v]);
                add_block(std::move(ids));
            }
        }
    }
    std::vector<bool> used(n); for(const auto& b:answer.blocks) for(int v:b.vertices) used[v]=true;
    for(int v=0;v<n;++v) if(!used[v]) answer.blocks.push_back({{v},{},{},{}});
    answer.block_cut_forest=Graph<int>(n+int(answer.blocks.size()));
    for(int i=0;i<int(answer.blocks.size());++i) for(int v:answer.blocks[i].vertices) answer.block_cut_forest.add_edge(v,n+i);
    return answer;
}
} // namespace cp
