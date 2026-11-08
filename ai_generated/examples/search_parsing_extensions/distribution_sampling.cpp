#include "heuristic_search/distribution_sampling.hpp"
#include <iostream>
using namespace cp;
int main() {
    AliasSampler sampler({1, 3, 2});
    for (std::size_t i = 0; i < sampler.size(); ++i)
        std::cout << (i ? " " : "") << sampler.thresholds()[i];
    std::cout << "\n";
    for (std::size_t i = 0; i < sampler.size(); ++i)
        std::cout << (i ? " " : "") << sampler.aliases()[i];
    std::cout << "\n";
}
