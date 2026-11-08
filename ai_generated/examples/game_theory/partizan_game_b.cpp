#include <iostream>
#include <vector>
#include "../../game_theory/partizan_game.hpp"

int main() {
    cp::PartizanGameArena games;
    int star = games.make({0}, {0});
    std::cout << (games.compare(star, 0) == cp::PartizanOrder::fuzzy) << " " << (games.compare(games.add(star, star), 0) == cp::PartizanOrder::equal) << "\n";
}
