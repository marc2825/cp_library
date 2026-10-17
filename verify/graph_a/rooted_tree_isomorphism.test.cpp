#define PROBLEM "https://judge.yosupo.jp/problem/rooted_tree_isomorphism_classification"
#include <iostream>
#include <map>
#include "tree/tree_hash.hpp"
int main(){int n;std::cin>>n;cp::Graph<int>g(n);for(int v=1;v<n;++v){int p;std::cin>>p;g.add_edge(p,v);}auto h=cp::rooted_tree_hash(g);std::map<cp::TreeHash,int>ids;std::vector<int>answer;for(const auto& x:h.hash){auto it=ids.find(x);if(it==ids.end())it=ids.emplace(x,int(ids.size())).first;answer.push_back(it->second);}std::cout<<ids.size()<<'\n';for(int i=0;i<n;++i){if(i)std::cout<<' ';std::cout<<answer[i];}std::cout<<'\n';}
