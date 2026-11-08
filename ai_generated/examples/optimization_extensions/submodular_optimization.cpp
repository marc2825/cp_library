#include "combinatorial_optimization/submodular_optimization.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto f = [](const std::vector<int> &s) {
        std::vector<bool> in(3);
        int value = 0;
        const int weight[] = {2, -3, 1};
        for (int e : s) {
            in[e] = true;
            value += weight[e];
        }
        value += in[0] != in[1];
        value += in[1] != in[2];
        return BigRational(value);
    };
    auto r = submodular_minimize(3, f);
    std::cout << r.value << " " << r.lower_bound << "\n";
}
