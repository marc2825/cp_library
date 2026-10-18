#include <iostream>
#include "range_query/sortable_segtree.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::SortableSegtree<long long,op,e> s({{2,10},{0,20},{1,30}},3); s.sort(0,3);
    std::cout << s.get(0).second << " " << s.prod(0,2) << "\n";
}
