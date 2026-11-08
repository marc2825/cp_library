#define PROBLEM "https://cses.fi/problemset/task/1730/"
#include <iostream>
#include <utility>
#include <vector>
#include "game_theory/nim.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int t;std::cin>>t;while(t--){int n;std::cin>>n;std::vector<long long>a(n);for(auto&x:a)std::cin>>x;std::cout<<(cp::nim_winning(a)?"first":"second")<<'\n';}}
