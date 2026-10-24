#define PROBLEM "https://judge.yosupo.jp/problem/dynamic_tree_subtree_add_subtree_sum"
#include <iostream>
#include <vector>
#include "../../tree/dynamic_subtree_aggregate.hpp"
#include "../../range_query/range_actions.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int n,q;std::cin>>n>>q;std::vector<long long>a(n);for(auto&x:a)std::cin>>x;
    using A=cp::AddSumAction<long long>;cp::DynamicSubtreeAggregate<A> t(A::values(a));
    for(int i=1;i<n;++i){int u,v;std::cin>>u>>v;t.link(u,v);}
    while(q--){int type,v,p;std::cin>>type>>v>>p;
        if(type==0){int w,x;std::cin>>w>>x;t.cut(v,p);t.link(w,x);}
        else if(type==1){long long x;std::cin>>x;t.subtree_apply(p,v,x);}
        else std::cout<<t.subtree_prod(p,v)->sum<<'\n';
    }
}
