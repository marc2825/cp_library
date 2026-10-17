#include <iostream>
#include <vector>
#include "combinatorics/polya.hpp"

int main() {
    cp::CycleIndex group({{0,1,2},{1,2,0},{2,0,1}});
    std::cout << group.colorings(2) << " " << group.evaluate({0,3,5,9}) << "\n";
}
