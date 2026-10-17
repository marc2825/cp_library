#define PROBLEM "https://judge.yosupo.jp/problem/lyndon_factorization"
#include <iostream>
#include <string>
#include "string_algorithm/lyndon_factorization.hpp"
int main(){std::string s;std::cin>>s;std::cout<<0;for(auto [l,r]:cp::lyndon_factorization(s)){(void)l;std::cout<<' '<<r;}std::cout<<'\n';}
