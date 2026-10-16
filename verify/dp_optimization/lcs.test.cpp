#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_10_C"
#include <iostream>
#include <string>
#include "dp_optimization/lcs.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int q;std::cin>>q;while(q--){std::string a,b;std::cin>>a>>b;std::cout<<cp::lcs_length(a,b)<<'\n';}}
