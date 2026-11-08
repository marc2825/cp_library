#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "linear_recurrence/all.hpp"

void print(const std::vector<int>& a) { std::cout << a.size(); for (int x : a) std::cout << ' ' << x; }
template <int P> void query(char op, const std::vector<int>& a, const std::vector<int>& b, std::size_t d, std::uint64_t n) {
    switch (op) {
        case 'K': print({cp::kitamasa<P>(a, b, n)}); break;
        case 'M': print({cp::linear_recurrence_matrix<P>(a, b, n)}); break;
        case 'C': print(cp::kitamasa_coefficients<P>(b, n)); break;
        case 'F': case 'E':
            if constexpr (cp::linear_recurrence_detail::prime(P)) {
                print(cp::berlekamp_massey<P>(a));
                if (op == 'E') std::cout << ' ' << cp::linear_recurrence_from_prefix<P>(a, n);
            } else std::cout << "PRIME";
            break;
        case 'R': case 'B':
            if constexpr (P == 17 || P == 97 || P == 998244353) {
                constexpr int root = P == 97 ? 5 : 3;
                print({op == 'R' ? cp::linear_recurrence_nth<P, root>(a, b, n) : cp::bostan_mori<P, root>(a, b, n)});
            } else std::cout << "NTT";
            break;
        case 'P': case 'A': {
            if (a.size() != d * d) throw std::invalid_argument("flat matrix dimension mismatch");
            std::vector<std::vector<int>> matrix(d, std::vector<int>(d));
            for (std::size_t i = 0; i < d; ++i) for (std::size_t j = 0; j < d; ++j) matrix[i][j] = a[i * d + j];
            if (op == 'A') print(cp::matrix_power_apply<P>(matrix, b, n));
            else {
                const auto power = cp::matrix_power_mod<P>(matrix, n); std::vector<int> flat;
                for (const auto& row : power) flat.insert(flat.end(), row.begin(), row.end());
                print(flat);
            }
            break;
        }
        default: throw std::invalid_argument("unknown operation");
    }
    std::cout << '\n';
}
int main() {
    char op; int p; std::size_t na, nb, d; std::uint64_t n;
    while (std::cin >> op >> p >> na >> nb >> d >> n) {
        std::vector<int> a(na), b(nb); for (auto& x : a) std::cin >> x; for (auto& x : b) std::cin >> x;
        try {
            switch (p) {
                case 1: query<1>(op, a, b, d, n); break;
                case 2: query<2>(op, a, b, d, n); break;
                case 12: query<12>(op, a, b, d, n); break;
                case 17: query<17>(op, a, b, d, n); break;
                case 97: query<97>(op, a, b, d, n); break;
                case 1000000007: query<1000000007>(op, a, b, d, n); break;
                case 2000000000: query<2000000000>(op, a, b, d, n); break;
                default: query<998244353>(op, a, b, d, n); break;
            }
        } catch (const std::length_error&) { std::cout << "LENGTH\n"; }
          catch (const std::domain_error&) { std::cout << "DOMAIN\n"; }
          catch (const std::invalid_argument&) { std::cout << "INVALID\n"; }
    }
}
