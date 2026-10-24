#include <iostream>
#include "../../range_query/kinetic_segtree.hpp"
int main(){
    cp::KineticSegtree tree({{1,0},{-1,10}});
    std::cout<<*tree.next_event()<<'\n';
    tree.advance(6); auto result=*tree.minimum(0,2);
    std::cout<<result.index<<' '<<static_cast<long long>(result.value)<<'\n';
}
