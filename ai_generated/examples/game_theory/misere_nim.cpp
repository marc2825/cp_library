#include <iostream>
#include <vector>
#include "game_theory/misere_nim.hpp"
int main(){
    std::vector<int> piles{1,1,5};
    std::cout<<cp::misere_nim_winning(piles)<<'\n';
    if(auto m=cp::misere_nim_winning_move(piles))
        std::cout<<m->pile<<' '<<m->remaining<<' '<<m->removed<<'\n';
    const std::vector<int> empty;
    std::cout<<cp::misere_nim_winning(empty)<<' '<<bool(cp::misere_nim_winning_move(empty))<<'\n';
}
