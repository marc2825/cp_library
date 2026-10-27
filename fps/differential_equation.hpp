#pragma once
#include "exp.hpp"
#include "inv.hpp"
#include "polynomial.hpp"
namespace cp {
// y'=a(x)y+b(x), y(0)=initial, modulo x^n.
template <int Mod = 998244353, int Root = 3>
std::vector<int> fps_solve_linear_ode(const std::vector<int> &a, const std::vector<int> &b, int initial,
                                      std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::check_precision<Mod, Root>(n);
    if (n > static_cast<std::size_t>(Mod))
        throw std::domain_error("ODE precision exceeds characteristic");
    if (!n)
        return {};
    auto integral = polynomial_integral<Mod>(fps_truncate<Mod>(a, n - 1));
    auto e = fps_exp<Mod, Root>(integral, n);
    auto rhs = fps_detail::product_prefix<Mod, Root>(b, fps_inv<Mod, Root>(e, n), n - 1);
    auto c = polynomial_integral<Mod>(rhs);
    c[0] = F::norm(initial);
    return fps_detail::product_prefix<Mod, Root>(e, c, n);
}
// y'=sum_j coefficients[j](x)y^j. Newton linearization, fixed initial value.
template <int Mod = 998244353, int Root = 3>
std::vector<int> fps_solve_polynomial_ode(const std::vector<std::vector<int>> &coefficients, int initial,
                                          std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::check_precision<Mod, Root>(n);
    if (n > static_cast<std::size_t>(Mod))
        throw std::domain_error("ODE precision exceeds characteristic");
    if (!n)
        return {};
    std::vector<int> y{F::norm(initial)};
    while (y.size() < n) {
        auto target = std::min(n, 2 * y.size());
        auto yn = fps_truncate<Mod>(y, target);
        std::vector<int> value(target - 1), derivative(target - 1), power(target - 1);
        power[0] = 1;
        for (std::size_t j = 0; j < coefficients.size(); ++j) {
            auto term = fps_detail::product_prefix<Mod, Root>(coefficients[j], power, target - 1);
            value = polynomial_add<Mod>(value, term);
            if (j + 1 < coefficients.size()) {
                term = fps_detail::product_prefix<Mod, Root>(coefficients[j + 1], power, target - 1);
                for (auto &x : term)
                    x = F::mul(x, static_cast<int>((j + 1) % Mod));
                derivative = polynomial_add<Mod>(derivative, term);
                power = fps_detail::product_prefix<Mod, Root>(power, yn, target - 1);
            }
        }
        auto residual = polynomial_subtract<Mod>(value, polynomial_derivative<Mod>(yn));
        auto correction = fps_solve_linear_ode<Mod, Root>(derivative, residual, 0, target);
        y = polynomial_add<Mod>(yn, correction);
    }
    return y;
}
} // namespace cp
