#define PROBLEM "https://atcoder.jp/contests/dp/tasks/dp_k"
#include <iostream>
#include <utility>
#include <vector>
#include "game_theory/dag_game.hpp"
int main(){std::ios::sync_with_stdio(false);std::cin.tie(nullptr);int n,k;std::cin>>n>>k;std::vector<int>allowed(n);for(auto&x:allowed)std::cin>>x;std::vector<std::vector<int>>moves(k+1);for(int v=0;v<=k;++v)for(int take:allowed)if(take<=v)moves[v].push_back(v-take);cp::DAGGame game(std::move(moves));std::cout<<(game.winning(k)?"First":"Second")<<'\n';}
