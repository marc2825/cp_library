#define PROBLEM "https://judge.yosupo.jp/problem/zalgorithm"
#include <iostream>
#include <string>
#include "string_algorithm/z_algorithm.hpp"
int main(){std::string s;std::cin>>s;auto z=cp::z_algorithm(s);for(std::size_t i=0;i<z.size();++i)std::cout<<(i?" ":"")<<z[i];std::cout<<'\n';}
