#include <iostream>
#include "graph_basic/segment_tree_graph.hpp"
#include "graph_basic/dijkstra.hpp"
int main() {
    cp::SegmentTreeGraph<> builder(4);
    builder.add_point_to_range(0,1,3,2);
    builder.add_range_to_point(1,3,3,5);
    builder.add_range_to_range(0,2,2,4,4);
    auto result=cp::dijkstra(builder.graph(),0);
    for (int v=0;v<4;++v) std::cout << result.dist[v] << (v==3?'\n':' ');
}
