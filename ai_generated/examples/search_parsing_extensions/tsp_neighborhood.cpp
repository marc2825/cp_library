#include "heuristic_search/tsp_neighborhood.hpp"
#include <iostream>
using namespace cp;
int main() {
    std::vector<int> tour{0, 1, 2, 3};
    const long long costs[4][4] = {{0, 8, 4, 4}, {8, 0, 4, 4}, {4, 4, 0, 8}, {4, 4, 8, 0}};
    auto distance = [&](int a, int b) { return costs[a][b]; };
    TspTwoOpt move{1, 3};
    std::cout << two_opt_delta(tour, move, distance) << " ";
    apply_two_opt(tour, move);
    std::cout << tsp_tour_cost(tour, distance) << " ";
    undo_two_opt(tour, move);
    std::cout << (tour == std::vector<int>{0, 1, 2, 3}) << "\n";
}
