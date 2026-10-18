#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/graphic_matroid.hpp"

int main() {
    cp::GraphicMatroid forest(3, {{0, 1}, {1, 2}, {2, 0}});
    std::cout << forest.independent({0, 1}) << " " << forest.independent({0, 1, 2}) << "\n";
}
