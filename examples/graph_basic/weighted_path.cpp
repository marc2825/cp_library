#include <iostream>
#include "graph_basic/weighted_path.hpp"
int main() {
    cp::WeightedPath<long long> p{5,{0,1},{7}};
    std::cout<<p.cost<<" "<<p.vertices.size()<<" "<<p.edge_ids[0]<<"\n";
}
