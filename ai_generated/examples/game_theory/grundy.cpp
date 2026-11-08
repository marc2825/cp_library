#include <iostream>
#include <vector>
#include "game_theory/grundy.hpp"
int main(){
    std::cout<<cp::grundy_mex({0,2,2,1000000000})<<'\n';
    const std::vector<std::vector<int>> moves{{},{0},{0,1},{1,2}};
    auto r=cp::calculate_grundy(moves);
    for(auto g:r.values)std::cout<<g<<' ';
    std::cout<<'\n';
}
