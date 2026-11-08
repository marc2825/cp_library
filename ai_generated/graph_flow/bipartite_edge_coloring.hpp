#pragma once
// 説明: docs/graph_flow/bipartite_edge_coloring.md
#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

namespace cp {
struct BipartiteEdgeColoringResult { int color_count=0; std::vector<int> color; };
// 多重辺可。2色の交互路を反転し、最大次数Delta色で彩色する。
inline BipartiteEdgeColoringResult bipartite_edge_coloring(int left_size,int right_size,const std::vector<std::pair<int,int>>& edges) {
    assert(left_size>=0 && right_size>=0); std::vector<int> degree(left_size+right_size);
    for(auto [u,v]:edges) { assert(0<=u && u<left_size && 0<=v && v<right_size); ++degree[u]; ++degree[left_size+v]; }
    int delta=degree.empty()?0:*std::max_element(degree.begin(),degree.end());
    std::vector<std::vector<int>> table(left_size+right_size,std::vector<int>(delta,-1));
    BipartiteEdgeColoringResult result{delta,std::vector<int>(edges.size(),-1)};
    auto assign=[&](int id,int color) { auto [u,v]=edges[id]; table[u][color]=id; table[left_size+v][color]=id; result.color[id]=color; };
    for(int id=0;id<int(edges.size());++id) {
        auto [u,v]=edges[id]; v+=left_size; int a=0,b=0;
        while(table[u][a]>=0) ++a;
        while(table[v][b]>=0) ++b;
        if(a!=b) {
            std::vector<int> path; int vertex=v,color=a;
            while(table[vertex][color]>=0) {
                int e=table[vertex][color]; path.push_back(e); auto [x,y]=edges[e]; y+=left_size;
                vertex=vertex==x?y:x; color=color==a?b:a;
            }
            for(int e:path) { auto [x,y]=edges[e]; table[x][result.color[e]]=-1; table[left_size+y][result.color[e]]=-1; }
            for(int e:path) assign(e,result.color[e]==a?b:a);
        }
        assign(id,a);
    }
    return result;
}
} // namespace cp
