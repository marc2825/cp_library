#include "computational_geometry/planar_embedding.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto e = rational::planar_embedding({{0, 0}, {6, 0}, {6, 6}, {0, 6}, {2, 2}, {4, 2}, {4, 4}, {2, 4}},
                                        {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}});
    std::cout << e.faces.size() << " " << e.dual_edges.size() << "\n";
}
