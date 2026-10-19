#include <iostream>
#include <string>
#include <vector>
#include "basic_utility/grid.hpp"
#include "graph/grid_bfs.hpp"
int main() {
    std::vector<std::string> grid{"abc", "def"};
    for (const auto& row : cp::grid_rotate_clockwise(grid)) std::cout << row << "\n";
    std::cout << cp::grid_shape(grid).neighbors4(0, 1).size() << "\n";
    std::cout << cp::grid_bfs({"...", ".#."}, 0, 0)[1][2] << "\n";
}
