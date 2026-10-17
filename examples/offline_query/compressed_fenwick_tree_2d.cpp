#include <iostream>
#include "offline_query/compressed_fenwick_tree_2d.hpp"
int main() {
    cp::CompressedFenwickTree2D<long long> s({{1,2},{100,200}}); s.add(1,2,7);
    std::cout<<s.sum(0,0,50,50)<<"\n";
}
