#define PROBLEM "https://judge.yosupo.jp/problem/suffixarray"
#include <iostream>
#include <string>
#include "string_algorithm/suffix_array.hpp"
int main(){std::string s;std::cin>>s;auto sa=cp::suffix_array(s);for(std::size_t i=0;i<sa.size();++i)std::cout<<(i?" ":"")<<sa[i];std::cout<<'\n';}
