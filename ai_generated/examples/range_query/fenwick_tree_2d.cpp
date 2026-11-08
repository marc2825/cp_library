#include <iostream>
#include "range_query/fenwick_tree_2d.hpp"
int main() {
    cp::FenwickTree2D<long long> s(4,5); s.add(1,2,7); s.add(3,4,9);
    std::cout<<s.sum(0,0,3,4)<<"\n";
}
