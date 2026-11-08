#define PROBLEM "https://judge.yosupo.jp/problem/persistent_unionfind"
#include <iostream>
#include <vector>
#include "union_find/persistent_disjoint_set_union.hpp"
int main(){
    int n,q;std::cin>>n>>q;cp::PersistentDisjointSetUnion d(n);std::vector<int> roots(q+1,0);
    for(int i=0;i<q;++i){int type,k,u,v;std::cin>>type>>k>>u>>v;int root=roots[k+1];
        if(type==0)roots[i+1]=d.merge(root,u,v);
        else{roots[i+1]=root;std::cout<<d.same(root,u,v)<<'\n';}
    }
}
