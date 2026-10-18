#include <iostream>
#include "static_query/weighted_wavelet_matrix.hpp"
int main() {
    cp::WeightedWaveletMatrix<int> s({3,1,2},{10,20,30});
    std::cout << s.range_sum(0,3,1,3) << " " << s.sum_smallest(0,3,2) << "\n";
}
