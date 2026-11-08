#pragma once
// 説明: docs/graph_combinatorics/dag_minimum_path_cover.md
#include <vector>
#include "../graph/graph.hpp"
#include "../graph_connectivity/dag_reachability.hpp"
#include "../graph_flow/bipartite_matching.hpp"
namespace cp {
struct DAGPathCoverResult { std::vector<std::vector<int>> paths,edge_ids; int size() const {return static_cast<int>(paths.size());} };
template<class T> DAGPathCoverResult dag_minimum_path_cover(const Graph<T>& g) {
    detail::reachability_topological_order(g); BipartiteMatching matching(g.size(),g.size());
    for(auto e:g.edges()) matching.add_edge(e.from,e.to);
    auto match=matching.maximum_matching(); DAGPathCoverResult result;
    for(int start=0;start<g.size();++start) if(match.right_match[start]==-1) {
        result.paths.push_back({}); result.edge_ids.push_back({});
        for(int v=start;v!=-1;v=match.left_match[v]) {
            result.paths.back().push_back(v); int to=match.left_match[v];
            if(to!=-1) for(auto e:g[v]) if(e.to==to) {result.edge_ids.back().push_back(e.idx);break;}
        }
    }
    return result;
}
} // namespace cp
