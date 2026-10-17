#include <iostream>
#include "string_algorithm/rolling_hash_2d.hpp"
int main() {
    cp::RollingHash2D a({"abcd", "efgh", "ijkl"}), b({"fg", "jk"});
    std::cout << a.equal(b, 1, 3, 1, 3, 0, 2, 0, 2) << '\n';
    const auto empty = a.get(1, 1, 0, 4);
    std::cout << empty[0] << ' ' << empty[1] << '\n';
}
