#include <iostream>
#include <vector>
#include "../../convolution/relaxed_convolution.hpp"

int main() {
    cp::RelaxedConvolution<> product(3);
    std::cout << product.append(1, 4) << " " << product.append(2, 5) << " " << product.append(3, 6) << "\n";
}
