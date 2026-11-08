#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "set_function/all.hpp"

struct Mod12 {
    int value;
    explicit Mod12(long long x = 0) : value(static_cast<int>((x % 12 + 12) % 12)) {}
    Mod12& operator+=(Mod12 b) { value = (value + b.value) % 12; return *this; }
    Mod12& operator-=(Mod12 b) { value = (value - b.value + 12) % 12; return *this; }
};
template <class T> void query(char op, std::size_t n) {
    std::vector<T> f(n);
    for (auto& x : f) {
        if constexpr (std::is_unsigned_v<T>) { unsigned long long v; std::cin >> v; x = static_cast<T>(v); }
        else { long long v; std::cin >> v; x = T(v); }
    }
    switch (op) {
        case 'Z': cp::subset_zeta_transform(f); break;
        case 'M': cp::subset_mobius_transform(f); break;
        case 'U': cp::superset_zeta_transform(f); break;
        case 'V': cp::superset_mobius_transform(f); break;
        case 'S': cp::sos_dp(f); break;
        case 'L': case 'H': case 'O':
            if constexpr (std::is_integral_v<T>) {
                if (op == 'L') cp::sos_dp(f, [](T a, T b) { return std::min(a, b); });
                if (op == 'H') cp::sos_dp(f, [](T a, T b) { return std::max(a, b); });
                if (op == 'O') cp::sos_dp(f, [](T a, T b) { return T(a | b); });
            } else throw std::invalid_argument("unsupported test aggregation");
            break;
        default: throw std::invalid_argument("unknown test operation");
    }
    std::cout << n;
    for (const auto& x : f) {
        if constexpr (std::is_same_v<T, Mod12>) std::cout << ' ' << x.value;
        else if constexpr (std::is_unsigned_v<T>) std::cout << ' ' << static_cast<unsigned long long>(x);
        else std::cout << ' ' << static_cast<long long>(x);
    }
    std::cout << '\n';
}
int main() {
    char op;
    while (std::cin >> op) try {
        if (op == 'e' || op == 'g') {
            std::uint64_t mask; int bits, include; std::cin >> mask >> bits >> include;
            std::vector<std::uint64_t> masks;
            if (op == 'e') cp::for_each_submask(mask, [&](auto x) { masks.push_back(x); }, include != 0);
            else cp::for_each_supermask(mask, bits, [&](auto x) { masks.push_back(x); });
            std::cout << masks.size(); for (auto x : masks) std::cout << ' ' << x; std::cout << '\n';
        } else {
            char kind; std::size_t n; std::cin >> kind >> n;
            if (kind == 's') query<long long>(op, n);
            else if (kind == 'u') query<unsigned long long>(op, n);
            else if (kind == 'b') query<signed char>(op, n);
            else query<Mod12>(op, n);
        }
    } catch (const std::overflow_error&) { std::cout << "OVERFLOW\n"; }
      catch (const std::invalid_argument&) { std::cout << "INVALID\n"; }
}
