#define PROBLEM "https://judge.yosupo.jp/problem/runenumerate"
#include <iostream>
#include <string>
#include "../../string_algorithm/runs_enumeration.hpp"
int main(){std::string s;std::cin>>s;const auto runs=cp::runs_enumeration(s);std::cout<<runs.size()<<'\n';for(const auto&r:runs)std::cout<<r.period<<' '<<r.left<<' '<<r.right<<'\n';}
