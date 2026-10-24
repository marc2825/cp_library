#include <iostream>
#include "../../union_find/fully_dynamic_connectivity.hpp"
int main(){
    cp::FullyDynamicConnectivity graph(4);
    int a=graph.add_edge(0,1); graph.add_edge(1,2);
    int c=graph.add_edge(0,2); graph.remove_edge(a);
    std::cout<<graph.connected(0,2)<<' '<<graph.component_size(0)<<'\n';
    graph.remove_edge(c);
    std::cout<<graph.connected(0,2)<<' '<<graph.component_count()<<'\n';
}
