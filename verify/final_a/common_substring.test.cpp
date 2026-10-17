#define PROBLEM "https://judge.yosupo.jp/problem/longest_common_substring"
#include <iostream>
#include <string>
#include "string_algorithm/longest_common_substring.hpp"
int main(){std::string a,b;std::cin>>a>>b;const auto r=cp::longest_common_substring(a,b);std::cout<<r.first_left<<' '<<r.first_right()<<' '<<r.second_left<<' '<<r.second_right()<<'\n';}
