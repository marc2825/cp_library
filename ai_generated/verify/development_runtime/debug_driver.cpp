#include <iostream>
#include <string>
#include <vector>
#include "basic_utility/debug.hpp"
int main() {
    int count;
    while (std::cin >> count) {
        std::vector<std::vector<long long>> matrix;
        for (int i = 0, n; i < count; ++i) {
            std::cin >> n;
            std::vector<long long> row(n);
            for (auto& value : row) std::cin >> value;
            matrix.push_back(row);
        }
        std::cout << cp::debug_string(matrix) << '\n';
    }
}
