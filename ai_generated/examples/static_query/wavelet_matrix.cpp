#include <iostream>
#include "static_query/wavelet_matrix.hpp"
int main() {
    cp::WaveletMatrix<int> w({5, 1, 4, 1, 3});
    std::cout << *w.kth_smallest(1, 5, 2) << ' ' << w.count_less(0, 5, 4) << ' ' << *w.predecessor(0, 5, 3) << '\n';
}
