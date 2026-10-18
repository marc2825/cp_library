#include <iostream>
#include "tree/static_top_tree.hpp"
struct SumSpec {
    using Vertex=long long; using Path=long long; using Point=long long;
    Point point_identity() const { return 0; }
    Point rake(Point a,Point b) const { return a+b; }
    Path add_vertex(Point a,Vertex v,int) const { return a+v; }
    Path compress(Path a,Path b,cp::Edge<long long>) const { return a+b; }
    Point to_point(Path a,cp::Edge<long long>) const { return a; }
};
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    cp::StaticTopTreeDP<SumSpec> tree(g,{1,2,3}); tree.set_vertex(1,5);
    std::cout << tree.component_prod(0) << '\n';
}
