#include <iostream>
#include "../../range_query/dynamic_wavelet_matrix.hpp"
int main(){
    cp::DynamicWaveletMatrix matrix(std::vector<std::uint64_t>{5,1,4,1},3);
    matrix.insert(2,7); matrix.set(0,2);
    std::cout<<matrix.rank(1,5)<<' '<<*matrix.kth_smallest(0,5,2)<<' '<<matrix.select(1,1)<<'\n';
    auto removed=matrix.erase(2);
    std::cout<<removed<<' '<<matrix.range_freq(0,4,1,4)<<'\n';
}
