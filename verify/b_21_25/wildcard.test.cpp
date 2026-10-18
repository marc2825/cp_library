#define PROBLEM "https://judge.yosupo.jp/problem/wildcard_pattern_matching"
#include <iostream>
#include <string>
#include "../../string_algorithm/wildcard_pattern_matching.hpp"
int main(){std::string s,t;std::cin>>s>>t;std::string answer(s.size()-t.size()+1,'0');for(int i:cp::wildcard_pattern_matching(s,t,'*'))answer[i]='1';std::cout<<answer<<'\n';}
