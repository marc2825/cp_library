#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_14_B"
#include <iostream>
#include <string>
#include "string_algorithm/kmp.hpp"
int main(){std::string s,p;std::cin>>s>>p;for(int i:cp::kmp_search(s,p))std::cout<<i<<'\n';}
