#include <iostream>
#include "offline_query/mo_on_tree.hpp"
int main() {
    cp::Graph<int> g(4); g.add_edge(0,1); g.add_edge(1,2); g.add_edge(1,3);
    cp::MoOnTree mo(g); mo.add_query(2,3); int sum=0;
    mo.run([&](int v){sum+=v;},[&](int v){sum-=v;},[&](int){std::cout<<sum<<"\n";});
}
