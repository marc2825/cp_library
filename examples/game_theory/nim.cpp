#include <iostream>
#include <vector>
#include "game_theory/nim.hpp"
int main(){
    std::vector<unsigned long long> piles{3,4,5};
    std::cout<<cp::nim_sum(piles)<<' '<<cp::nim_winning(piles)<<'\n';
    if(auto m=cp::nim_winning_move(piles))
        std::cout<<m->pile<<' '<<m->remaining<<' '<<m->removed<<'\n';
}
