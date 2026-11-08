#include <iostream>
#include <vector>
#include "../../heuristic_search/neighborhood_manager.hpp"

int main() {
    cp::NeighborhoodManager manager({0, 1}); cp::Random random(1);
    std::cout << manager.select(random) << "\n";
}
