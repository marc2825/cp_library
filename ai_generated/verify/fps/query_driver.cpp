#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "fps/all.hpp"

void print(const std::vector<int>& a) { std::cout << a.size(); for (int x : a) std::cout << ' ' << x; }
template <int P, int Root> void query(char op, const std::vector<int>& a, const std::vector<int>& b, std::uint64_t k, long long shift, std::size_t n) {
    switch (op) {
        case 'A': print(cp::polynomial_add<P>(a, b)); break;
        case 'S': print(cp::polynomial_subtract<P>(a, b)); break;
        case 'M': print(cp::polynomial_multiply<P, Root>(a, b)); break;
        case 'V': print(cp::polynomial_derivative<P>(a)); break;
        case 'I': print(cp::polynomial_integral<P>(a)); break;
        case 'T': print(cp::fps_truncate<P>(a, n)); break;
        case 'F': print(cp::fps_inv<P, Root>(a, n)); break;
        case 'L': print(cp::fps_log<P, Root>(a, n)); break;
        case 'E': print(cp::fps_exp<P, Root>(a, n)); break;
        case 'P': print(cp::fps_pow<P, Root>(a, k, n)); break;
        case 'Q': { const auto r = cp::fps_sqrt<P, Root>(a, n); if (r) print(*r); else std::cout << "NONE"; break; }
        case 'D': { const auto [q, r] = cp::polynomial_divmod<P, Root>(a, b); print(q); std::cout << ' '; print(r); break; }
        case 'X': print(cp::multipoint_evaluation<P, Root>(a, b)); break;
        case 'H': print(cp::polynomial_interpolation<P, Root>(a, b)); break;
        case 'C': print(cp::taylor_shift<P, Root>(a, shift)); break;
        case 'B': print({cp::bostan_mori<P, Root>(a, b, k)}); break;
        case 'R': print({cp::linear_recurrence_nth<P, Root>(a, b, k)}); break;
        default: throw std::invalid_argument("unknown operation");
    }
    std::cout << '\n';
}
int main() {
    char op; int p; std::size_t na, nb, n; std::uint64_t k; long long shift;
    while (std::cin >> op >> p >> na >> nb >> k >> shift >> n) {
        std::vector<int> a(na), b(nb); for (auto& x : a) std::cin >> x; for (auto& x : b) std::cin >> x;
        try {
            if (p == 17) query<17, 3>(op, a, b, k, shift, n);
            else if (p == 97) query<97, 5>(op, a, b, k, shift, n);
            else if (p == 1224736769) query<1224736769, 3>(op, a, b, k, shift, n);
            else query<998244353, 3>(op, a, b, k, shift, n);
        } catch (const std::length_error&) { std::cout << "LENGTH\n"; }
          catch (const std::domain_error&) { std::cout << "DOMAIN\n"; }
          catch (const std::invalid_argument&) { std::cout << "INVALID\n"; }
    }
}
