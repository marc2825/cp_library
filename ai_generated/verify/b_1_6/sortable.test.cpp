#define PROBLEM "https://judge.yosupo.jp/problem/point_set_range_sort_range_composite"
#include <iostream>
#include <utility>
#include <vector>
#include "range_query/sortable_segtree.hpp"
#include "modint.hpp"
struct BAffine { BModInt a,b; };
BAffine compose(BAffine left,BAffine right) { return {right.a*left.a,right.a*left.b+right.b}; }
BAffine identity() { return {1,0}; }
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    int n,q; std::cin>>n>>q; std::vector<std::pair<int,BAffine>> values;
    for(int i=0;i<n;++i) { int key; long long a,b; std::cin>>key>>a>>b; values.push_back({key,{a,b}}); }
    cp::SortableSegtree<BAffine,compose,identity> tree(values,1000000001);
    while(q--) {
        int type,l,r; std::cin>>type>>l>>r;
        if(type==0) { long long a,b; std::cin>>a>>b; tree.set(l,r,{a,b}); }
        else if(type==1) { long long x; std::cin>>x; auto f=tree.prod(l,r); std::cout<<(f.a*BModInt(x)+f.b).value<<'\n'; }
        else tree.sort(l,r,type==3);
    }
}
