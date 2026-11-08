#include <iostream>
#include <vector>
#include "dp_optimization/aliens_trick.hpp"

int main() {
    auto oracle=[](long long p) {
        cp::PenalizedDPResult best{1000000,0};
        for (int k=0;k<=6;++k) {
            long long value=(k-3)*(k-3)+p*k;
            if (value<=best.value) best={value,k};
        }
        return best;
    };
    auto result=cp::aliens_trick(2,-20,20,oracle);
    std::cout << result->value << "\n";
}
