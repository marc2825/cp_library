#include <iostream>
#include <vector>
#include "../../string_algorithm/suffix_tree.hpp"

int main() {
    cp::SuffixTree tree("banana");
    auto positions = tree.occurrences("ana");
    std::cout << positions[0] << " " << positions[1] << "\n";
}
