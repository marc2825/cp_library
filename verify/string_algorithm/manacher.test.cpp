#define PROBLEM "https://judge.yosupo.jp/problem/enumerate_palindromes"
#include <iostream>
#include <string>
#include "string_algorithm/manacher.hpp"
int main(){std::string s;std::cin>>s;auto r=cp::manacher(s);for(std::size_t i=0;i<s.size();++i){if(i)std::cout<<' '<<2*r.even[i]<<' ';std::cout<<2*r.odd[i]-1;}std::cout<<'\n';}
