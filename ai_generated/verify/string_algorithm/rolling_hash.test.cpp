#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_14_B"
#include <iostream>
#include <string>
#include "string_algorithm/rolling_hash.hpp"
int main(){std::string s,p;std::cin>>s>>p;cp::RollingHash hash(s);auto pattern=hash.hash_string(p);if(p.size()<=s.size())for(std::size_t i=0;i<=s.size()-p.size();++i)if(hash.get(static_cast<int>(i),static_cast<int>(i+p.size()))==pattern)std::cout<<i<<'\n';}
