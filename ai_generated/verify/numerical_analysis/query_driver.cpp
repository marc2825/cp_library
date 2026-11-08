#include "numerical_analysis/all.hpp"
#include <iomanip>
#include <iostream>
#include <string>
using namespace cp::numerical;
std::vector<long double> read(std::size_t n) {
    std::vector<long double> a(n);
    for (auto &x : a)
        std::cin >> x;
    return a;
}
int main() {
    std::cout << std::setprecision(24);
    int count;
    std::cin >> count;
    while (count--) {
        char code;
        std::cin >> code;
        if (code == 'Q') {
            long double a, b, c;
            std::cin >> a >> b >> c;
            auto r = quadratic_roots(a, b, c);
            std::cout << status_name(r.status) << ' ' << r.all_real << ' ' << r.roots.size();
            for (auto x : r.roots)
                std::cout << ' ' << x;
        } else if (code == 'R') {
            int method;
            long double a;
            std::cin >> method >> a;
            RootOptions o;
            o.value_tolerance = 1e-15L;
            o.x_tolerance = {1e-15L, 1e-15L};
            auto f = [=](long double x) { return x * x - a; };
            auto d = [](long double x) { return 2 * x; };
            auto r = method == 0   ? bisection(f, 0, std::max(1.L, a), o)
                     : method == 1 ? brent(f, 0, std::max(1.L, a), o)
                                   : newton(f, d, std::max(1.L, a), o);
            std::cout << status_name(r.status) << ' ' << r.root;
        } else if (code == 'I') {
            std::size_t n;
            int method;
            long double a, b;
            std::cin >> method >> n >> a >> b;
            const auto coefficients = read(n);
            auto f = [&](long double x) {
                long double y = 0;
                for (auto i = n; i > 0; --i)
                    y = y * x + coefficients[i - 1];
                return y;
            };
            auto r = method ? adaptive_simpson(f, a, b) : simpson(f, a, b, 16);
            std::cout << status_name(r.status) << ' ' << r.value;
        } else if (code == 'T') {
            std::size_t n;
            std::cin >> n;
            auto l = read(n ? n - 1 : 0), d = read(n), u = read(n ? n - 1 : 0), b = read(n);
            auto r = solve_tridiagonal(l, d, u, b, {0, 1e-14L});
            std::cout << status_name(r.status);
            for (auto x : r.solution)
                std::cout << ' ' << x;
        } else if (code == 'P') {
            std::size_t n;
            std::cin >> n;
            auto r = real_polynomial_roots(read(n));
            std::cout << status_name(r.status) << ' ' << r.distinct_roots;
            for (auto x : r.intervals)
                std::cout << ' ' << x.lower << ' ' << x.upper << ' ' << x.count << ' ' << x.value;
        } else if (code == 'S') {
            std::size_t n;
            std::cin >> n;
            auto values = read(n);
            std::cout << compensated_sum(values.begin(), values.end());
        } else if (code == 'L') {
            std::size_t n;
            std::cin >> n;
            std::cout << log_sum_exp(read(n));
        } else
            return 2;
        std::cout << '\n';
    }
}
