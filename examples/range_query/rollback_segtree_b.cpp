#include <iostream>
#include "range_query/rollback_segtree.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::RollbackSegtree<long long,op,e> s({1,2,3}); auto token=s.snapshot(); s.set(1,8); s.rollback(token);
    std::cout << s.all_prod() << "\n";
}
