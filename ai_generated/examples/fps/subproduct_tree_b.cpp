#include <iostream>
#include <vector>
#include "../../fps/subproduct_tree.hpp"

int main() {
    cp::SubproductTree<> tree({1, 2, 3});
    auto values = tree.evaluate({1, 0, 1});
    std::cout << values[2] << " " << tree.interpolate(values)[2] << "\n";
}
