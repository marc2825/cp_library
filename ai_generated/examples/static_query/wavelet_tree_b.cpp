#include <iostream>
#include "static_query/wavelet_tree.hpp"
int main() {
    cp::WaveletTree<int> s({4,1,4,2});
    std::cout << *s.kth_smallest(0,4,2) << "\n";
}
