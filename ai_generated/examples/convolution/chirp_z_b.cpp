#include <iostream>
#include <vector>
#include "../../convolution/chirp_z.hpp"

int main() {
    auto c = cp::chirp_z(std::vector<int>{1, 2, 1}, 1, 2, 3);
    std::cout << c[0] << " " << c[1] << " " << c[2] << "\n";
}
