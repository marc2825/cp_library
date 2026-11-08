#include <iostream>
#include "dp_optimization/myers_edit_distance.hpp"
int main() {
    std::cout << cp::myers_edit_distance("kitten", "sitting") << '\n';
}
