#include <iostream>
#include "basic_utility/maximum_subarray.hpp"
int main() {
    auto answer = cp::maximum_subarray(std::vector<long long>{-2, 3, -1, 3, -5});
    std::cout << answer->sum << ' ' << answer->left << ' ' << answer->right << '\n';
    std::cout << cp::maximum_subarray(std::vector<int>{-4, -2}, true)->sum << '\n';
}
