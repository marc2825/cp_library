#define PROBLEM "https://cses.fi/problemset/task/1715"
#include <array>
#include <iostream>
#include <string>
#include "combinatorics/factorials.hpp"
int main() {
    std::string s; std::cin >> s;
    cp::FactorialTable table(static_cast<int>(s.size()), 1000000007);
    std::array<int, 26> count{}; for (char c : s) ++count[c - 'a'];
    long long answer = table.factorial(static_cast<long long>(s.size()));
    for (int size : count) answer = cp::mod_multiply(answer, table.inverse_factorial(size), table.modulus());
    std::cout << answer << '\n';
}
