#include <iostream>
#include "static_query/disjoint_sparse_table.hpp"
long long add(long long a,long long b){return a+b;} long long zero(){return 0;}
int main() {
    cp::DisjointSparseTable<long long,add,zero> s({2,3,5,7});
    std::cout<<s.prod(1,4)<<" "<<s.prod(2,2)<<"\n";
}
