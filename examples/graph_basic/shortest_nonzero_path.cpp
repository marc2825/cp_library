#include <iostream>
#include "graph_basic/shortest_nonzero_path.hpp"
struct Parity {
    using value_type=int;
    static int identity() { return 0; }
    static int op(int a,int b) { return a^b; }
    static int inverse(int a) { return a; }
};
int main() {
    cp::Graph<int> g(3);
    g.add_edge(0,1,1); g.add_edge(1,2,1); g.add_edge(0,2,5);
    auto r=cp::shortest_nonzero_paths<Parity>(g,{1,1,1},0);
    std::cout << cp::optimization_to_string(*r.distance[2]) << ' ' << r.path(2).edge_ids.size() << '\n';
}
