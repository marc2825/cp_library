#include <iostream>
#include "range_query/dynamic_segtree.hpp"
long long sum(long long a,long long b) { return a+b; }
long long zero() { return 0; }
int main() {
    cp::DynamicSegtree<long long,sum,zero> a(10);
    a.set(2,5); a.set(8,7);
    auto saved=a;
    auto right=a.split(5);
    a.set(2,9);
    a.merge(right);
    std::cout << saved.all_prod() << " " << a.all_prod() << " " << right.all_prod() << "\n";
    std::cout << a.merged(saved).get(2) << "\n";
}
