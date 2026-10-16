#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_4_C"
#include <iostream>
#include <string>
#include "string_algorithm/trie.hpp"
int main(){int n;std::cin>>n;cp::Trie trie;while(n--){std::string op,s;std::cin>>op>>s;if(op=="insert")trie.insert(s);else std::cout<<(trie.contains(s)?"yes":"no")<<'\n';}}
