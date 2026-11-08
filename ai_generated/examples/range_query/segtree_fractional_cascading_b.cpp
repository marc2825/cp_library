#include <iostream>
#include "range_query/segtree_fractional_cascading.hpp"
int main() {
    cp::SegtreeFractionalCascading<int> s({4,1,4,2});
    std::cout << s.range_freq(0,4,2,5) << "\n";
}
