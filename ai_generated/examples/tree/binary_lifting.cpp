#include <iostream>
#include "tree/binary_lifting.hpp"
int main() {
    cp::BinaryLifting next({1, 2, 0, -1});
    std::cout << next.jump(0, 1000000000000000000ULL) << ' ' << next.jump(3, 1) << '\n';
}
