// Python の任意精度整数との比較用。一括入力でプロセス起動数を抑える。
#include <iostream>
#include "number_theory_basic/all.hpp"
int main() {
    std::ios::sync_with_stdio(false); std::cin.tie(nullptr);
    char operation; long long a, b, m;
    while (std::cin >> operation >> a >> b) {
        if (operation == 'g') {
            const auto e = cp::extended_gcd(a, b); const auto l = cp::lcm(a, b);
            std::cout << cp::gcd(a, b) << ' ' << e.gcd << ' ' << e.x << ' ' << e.y << ' ';
            if (l) std::cout << *l; else std::cout << -1;
        } else if (operation == 'i') {
            const auto x = cp::mod_inverse(a, b); std::cout << (x ? *x : -1);
        } else {
            std::cin >> m;
            std::cout << (operation == 'p' ? cp::mod_pow(a, b, m) : cp::mod_multiply(a, b, m));
        }
        std::cout << '\n';
    }
}
