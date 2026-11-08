#include <iostream>
#include "tree/contour_query.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    cp::ContourQuery<> q(g); q.add(0,5); q.add(2,7);
    std::cout << q.sum(1,1,2) << '\n';
}
