#include <iostream>
#include "range_query/dynamic_segtree.hpp"
long long add(long long a,long long b){return a+b;} long long zero(){return 0;}
int main() {
    cp::DynamicSegtree<long long,add,zero> s(1000000000000LL);
    s.set(7,3); s.set(900000000000LL,5); std::cout<<s.all_prod()<<"\n";
}
