#include "combinatorial_optimization/weighted_matroid_intersection.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto independent = [](std::vector<int> groups) {
        return [groups](const std::vector<int> &s) {
            std::vector<int> count(2);
            for (int e : s)
                if (++count[groups[e]] > 1)
                    return false;
            return true;
        };
    };
    auto r = weighted_matroid_intersection(std::vector<long long>{8, 7, 6, 1}, independent({0, 0, 1, 1}),
                                           independent({0, 1, 0, 1}));
    std::cout << optimization_to_string(r.weight) << "\n";
    for (std::size_t i = 0; i < r.selected.size(); i++)
        std::cout << (i ? " " : "") << r.selected[i];
    std::cout << "\n";
}
