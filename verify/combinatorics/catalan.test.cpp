#define PROBLEM "https://cses.fi/problemset/task/2064"
#include <iostream>
#include "combinatorics/catalan.hpp"
int main() {
    int length; std::cin >> length;
    if (length % 2) { std::cout << 0 << '\n'; return 0; }
    cp::FactorialTable table(length, 1000000007);
    std::cout << cp::catalan(table, length / 2) << '\n';
}
