#include <iostream>
#include "range_query/segtree_2d.hpp"
long long add(long long a,long long b){return a+b;} long long zero(){return 0;}
int main() {
    cp::Segtree2D<long long,add,zero> s(std::vector<std::vector<long long>>{{1,2},{3,4}});
    s.set(0,1,8); std::cout<<s.prod(0,0,2,2)<<"\n";
}
