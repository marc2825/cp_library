#pragma once
#include <vector>
namespace cp {
// 元グラフの論理辺ID。vertices.size()==edge_ids.size()+1。
template<class T> struct WeightedPath { T cost; std::vector<int> vertices,edge_ids; };
}
