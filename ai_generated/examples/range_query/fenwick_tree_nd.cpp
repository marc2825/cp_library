#include <iostream>
#include "../../range_query/fenwick_tree_nd.hpp"
int main(){
    cp::FenwickTreeND<long long,3> tree({{1,2,3},{4,5,6}});
    tree.add({1,2,3},7); tree.add({4,5,6},11);
    std::cout<<tree.prefix_sum({5,6,7})<<' '<<tree.sum({0,0,0},{3,4,5})<<'\n';
}
