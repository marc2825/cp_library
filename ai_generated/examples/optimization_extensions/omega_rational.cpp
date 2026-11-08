#include "game_theory/omega_rational.hpp"
#include <iostream>
using namespace cp;
int main() {
    const auto w = OmegaRational::omega(), epsilon = OmegaRational::infinitesimal();
    std::cout << ((w * w - 1) / (w - 1) == w + 1) << " " << (epsilon > 0) << " "
              << *(1 + epsilon).standard_part() << "\n";
}
