#include <iostream>
#include "combinatorics/stirling_small_prime.hpp"
int main() {
    cp::StirlingSmallPrime s(5);
    std::cout << s.first(1000000000003ULL, 3) << ' ' << s.second(1000000000003ULL, 3) << '\n';
}
