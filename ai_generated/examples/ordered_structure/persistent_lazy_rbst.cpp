#include <iostream>
#include "../../ordered_structure/persistent_lazy_rbst.hpp"
struct S{long long sum;int length;};
S op(S a,S b){return {a.sum+b.sum,a.length+b.length};}
S e(){return {0,0};}
S mapping(long long f,S x){return {x.sum+f*x.length,x.length};}
long long composition(long long f,long long g){return f+g;}
long long id(){return 0;}
int main(){
    cp::PersistentLazyRBST<S,op,e,long long,mapping,composition,id> tree({{1,1},{2,1},{3,1}});
    auto old=tree; tree.apply(0,2,10); tree.reverse(0,3);
    std::cout<<old.all_prod().sum<<' '<<tree.all_prod().sum<<' '<<tree.prod(0,2).sum<<'\n';
}
