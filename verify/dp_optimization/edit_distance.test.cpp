#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_E"
#include <iostream>
#include <string>
#include "dp_optimization/edit_distance.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);std::string a,b;std::cin>>a>>b;std::cout<<cp::edit_distance(a,b)<<'\n';}
