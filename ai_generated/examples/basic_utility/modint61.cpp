#include <iostream>
#include <string>
#include <vector>
#include "basic_utility/modint61.hpp"
int main() {
    using Mint = cp::ModInt61;
    std::string text = "ababa";
    Mint base = 911382323; // 固定値は再現用。hashの一致だけで文字列の同一性は保証しない。
    std::vector<Mint> prefix(text.size() + 1), power(text.size() + 1, Mint(1));
    for (std::size_t i = 0; i < text.size(); ++i) {
        prefix[i + 1] = prefix[i] * base + static_cast<unsigned char>(text[i]) + 1;
        power[i + 1] = power[i] * base;
    }
    auto hash = [&](std::size_t l, std::size_t r) { return prefix[r] - prefix[l] * power[r - l]; };
    std::cout << (hash(0, 3) == hash(2, 5)) << " " << (Mint(-1) * Mint(-1)) << "\n";
}
