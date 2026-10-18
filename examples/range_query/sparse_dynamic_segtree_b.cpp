#include <iostream>
#include "range_query/sparse_dynamic_segtree.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::SparseDynamicSegtree<long long,op,e> s(1LL<<40); s.set(123,7);
    std::cout << s.all_prod() << "\n";
}
