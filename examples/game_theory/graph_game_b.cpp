#include <iostream>
#include <vector>
#include "../../game_theory/graph_game.hpp"

int main() {
    auto result = cp::graph_game({{1}, {}, {2}});
    std::cout << (result.status[0] == cp::GraphGameStatus::win) << " " << (result.status[2] == cp::GraphGameStatus::draw) << "\n";
}
