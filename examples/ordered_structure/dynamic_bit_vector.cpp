#include <iostream>
#include "../../ordered_structure/dynamic_bit_vector.hpp"
int main(){
    cp::DynamicBitVector bits(std::vector<bool>{1,0,1,1});
    bits.insert(1,true); bits.erase(3);
    std::cout<<bits.rank(true,4)<<' '<<bits.select(false,0)<<' '<<bits.select(true,2)<<'\n';
}
