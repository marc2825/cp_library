#include <iostream>
#include "range_query/dual_segtree_2d.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::DualSegtree2D<long long,op,e> s(3,4); s.apply(0,2,1,4,5);
    std::cout << s.get(1,2) << "\n";
}
