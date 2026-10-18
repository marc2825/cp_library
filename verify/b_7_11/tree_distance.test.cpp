#define PROBLEM "https://judge.yosupo.jp/problem/frequency_table_of_tree_distance"
#include <iostream>
#include "tree/tree_distance_distribution.hpp"
int main(){int n;if(!(std::cin>>n))return 0;cp::Graph<int> g(n);for(int i=1;i<n;++i){int u,v;std::cin>>u>>v;g.add_edge(u,v);}auto a=cp::tree_distance_distribution(g);for(int i=1;i<n;++i)std::cout<<a[i]<<(i+1==n?'\n':' ');}
