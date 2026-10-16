#include <iostream>
#include <vector>
#include "combinatorial_optimization/hungarian.hpp"
int main(){
    std::vector<std::vector<long long>> cost{{4,1,3},{2,0,5},{3,2,2}};
    auto r=cp::hungarian(cost);
    if(r){std::cout<<r->cost<<'\n';for(auto j:r->row_to_column)std::cout<<j<<' ';std::cout<<'\n';}
}
