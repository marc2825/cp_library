#include <iostream>
#include "union_find/aggregate_disjoint_set_union.hpp"
long long add(long long a,long long b){return a+b;} long long zero(){return 0;}
int main() {
    cp::AggregateDisjointSetUnion<long long,add,zero> d(std::vector<long long>{2,3,7});
    d.merge(0,1); d.add(1,4); std::cout<<d.prod(0)<<"\n";
}
