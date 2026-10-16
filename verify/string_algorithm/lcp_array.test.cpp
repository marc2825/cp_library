#define PROBLEM "https://judge.yosupo.jp/problem/number_of_substrings"
#include <iostream>
#include <string>
#include "string_algorithm/suffix_array.hpp"
#include "string_algorithm/lcp_array.hpp"
int main(){std::string s;std::cin>>s;long long n=static_cast<long long>(s.size()),answer=n*(n+1)/2;for(int x:cp::lcp_array(s,cp::suffix_array(s)))answer-=x;std::cout<<answer<<'\n';}
