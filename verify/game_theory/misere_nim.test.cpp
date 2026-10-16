#define PROBLEM "https://www.hackerrank.com/challenges/misere-nim-1/problem"
#include <iostream>
#include <utility>
#include <vector>
#include "game_theory/misere_nim.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int t;std::cin>>t;while(t--){int n;std::cin>>n;std::vector<long long>a(n);for(auto&x:a)std::cin>>x;std::cout<<(cp::misere_nim_winning(a)?"First":"Second")<<'\n';}}
