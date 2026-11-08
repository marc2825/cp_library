#define PROBLEM "https://judge.yosupo.jp/problem/dynamic_tree_vertex_add_path_sum"
#include <iostream>
#include "tree/link_cut_tree.hpp"
long long op(long long a,long long b){return a+b;}long long e(){return 0;}
int main(){int n,q;std::cin>>n>>q;std::vector<long long>a(n);for(auto &x:a)std::cin>>x;cp::LinkCutTree<long long,op,e>t(a);for(int i=1;i<n;++i){int u,v;std::cin>>u>>v;t.link(u,v);}while(q--){int type;std::cin>>type;if(type==0){int u,v,w,x;std::cin>>u>>v>>w>>x;t.cut(u,v);t.link(w,x);}else if(type==1){int p;long long x;std::cin>>p>>x;t.set(p,t.get(p)+x);}else{int u,v;std::cin>>u>>v;std::cout<<*t.path_prod(u,v)<<'\n';}}}
