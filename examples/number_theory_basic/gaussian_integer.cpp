#include <iostream>
#include "number_theory_basic/gaussian_integer.hpp"

int main() {
    const cp::GaussianInteger a(17, 4), b(4, 1);
    const auto [q, r] = cp::gaussian_divmod(a, b);
    const auto g = cp::gaussian_gcd(a, b);
    std::cout << q.real << ' ' << q.imag << ' ' << r.real << ' ' << r.imag << '\n';
    std::cout << g.real << ' ' << g.imag << '\n';
}
