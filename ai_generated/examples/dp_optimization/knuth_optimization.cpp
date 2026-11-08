#include <iostream>
#include <vector>
#include "dp_optimization/knuth_optimization.hpp"

int main() {
    std::vector<long long> prefix{0,10,30,60};
    auto result=cp::knuth_optimization(3,[&](int l,int r) {return prefix[r]-prefix[l];});
    std::cout << result.value() << " " << result.merge_order().size() << "\n";
}
