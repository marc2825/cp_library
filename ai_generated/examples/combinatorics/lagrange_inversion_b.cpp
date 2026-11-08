#include <iostream>
#include "combinatorics/lagrange_inversion.hpp"
int main() {
    std::cout << cp::lagrange_inversion_coefficient(std::vector<int>{1,2,1},std::vector<int>{0,1},4) << '\n';
}
