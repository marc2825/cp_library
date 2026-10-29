#include "heuristic_search/index_set.hpp"
#include <iostream>
using namespace cp;
int main() {
    IndexSet selected(6);
    for (int e : {2, 4, 1})
        selected.insert(e);
    selected.erase(4);
    std::cout << selected.size() << "\n";
    for (std::size_t i = 0; i < selected.size(); ++i)
        std::cout << (i ? " " : "") << selected.at(i);
    std::cout << "\n";
    Random random(7);
    const auto sampled = selected.sample(random);
    std::cout << bool(sampled && selected.contains(*sampled)) << "\n";
}
