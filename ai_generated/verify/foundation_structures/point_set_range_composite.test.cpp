// PROBLEM https://judge.yosupo.jp/problem/point_set_range_composite
#include <vector>
#include "basic_utility/fast_io.hpp"
#include "basic_utility/static_modint.hpp"
#include "range_query/monoid_catalog.hpp"
int main(){
    cp::FastScanner in;cp::FastPrinter out;using Mint=cp::ModInt998244353;using M=cp::AffineFunctionMonoid<Mint>;
    int n=0,q=0;in.read(n,q);std::vector<M::S>a(n);long long x=0,y=0;
    for(auto&f:a){in.read(x,y);f={Mint(x),Mint(y)};}
    cp::MonoidSegtree<M> tree(a);
    while(q--){int op=0,l=0,r=0;in.read(op,l);
        if(op==0){in.read(x,y);tree.set(l,{Mint(x),Mint(y)});}
        else{in.read(r,x);out.writeln(tree.prod(l,r)(Mint(x)).val());}
    }
    out.flush();
}
