#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_14_D"
#include <iostream>
#include <string>
#include "string_algorithm/aho_corasick.hpp"
int main(){std::string text;int q;std::cin>>text>>q;cp::AhoCorasick ac;while(q--){std::string p;std::cin>>p;ac.insert(p);}ac.build();for(auto n:ac.count_occurrences(text))std::cout<<(n?1:0)<<'\n';}
