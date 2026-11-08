#include "heuristic_search/fast_annealing_acceptance.hpp"
#include <iostream>
using namespace cp;
int main() {
    FastAnnealingAcceptance accept(4);
    int count = 0;
    for (std::size_t i = 0; i < accept.thresholds().size(); ++i)
        count += accept.accept_index(.7L, 1, i);
    std::cout << count << " " << accept.thresholds().size() << "\n";
}
