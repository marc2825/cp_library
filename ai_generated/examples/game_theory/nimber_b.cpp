#include <iostream>
#include <vector>
#include "../../game_theory/nimber.hpp"

int main() {
    cp::Nimber a(3), b(5);
    std::cout << ((a * b) / b).value() << " " << (a + a).value() << "\n";
}
