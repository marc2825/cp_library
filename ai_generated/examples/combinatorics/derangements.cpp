#include <iostream>
#include <vector>
#include "combinatorics/derangements.hpp"

int main() {
    std::cout << cp::derangements(5)[5] << "\n";
}
