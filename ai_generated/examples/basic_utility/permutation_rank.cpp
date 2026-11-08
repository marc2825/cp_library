#include <iostream>
#include "basic_utility/permutation_rank.hpp"
int main() {
    auto p=cp::permutation_unrank(3,4);
    for(int x:p) std::cout<<x;
    std::cout<<" "<<cp::permutation_rank(p)<<"\n";
}
