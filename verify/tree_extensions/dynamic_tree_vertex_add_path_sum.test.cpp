#define PROBLEM "https://judge.yosupo.jp/problem/dynamic_tree_vertex_add_path_sum"
#include <iostream>
#include <vector>
#include "../../tree/dynamic_subtree_aggregate.hpp"
#include "../../range_query/range_actions.hpp"
int main(){
    std::ios::sync_with_stdio(false);std::cin.tie(nullptr);
    int n,q;std::cin>>n>>q;std::vector<long long>a(n);for(auto&x:a)std::cin>>x;
    using A=cp::AddSumAction<long long>;cp::DynamicSubtreeAggregate<A> t(A::values(a));
    for(int i=1;i<n;++i){int u,v;std::cin>>u>>v;t.link(u,v);}
    while(q--){int type,u,v;std::cin>>type>>u;
        if(type==0){int w,x;std::cin>>v>>w>>x;t.cut(u,v);t.link(w,x);}
        else if(type==1){long long x;std::cin>>x;auto value=t.get(u);value.sum+=x;t.set(u,value);}
        else{std::cin>>v;std::cout<<t.path_prod(u,v)->sum<<'\n';}
    }
}
