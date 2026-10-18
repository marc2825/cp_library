#include <iostream>
#include <vector>
#include "../../convolution/convolution_large.hpp"

int main() {
    auto c = cp::convolution_large<17, 3>(std::vector<int>(20, 1), std::vector<int>(20, 1), 8);
    std::cout << c.size() << " " << c[19] << "\n";
}
