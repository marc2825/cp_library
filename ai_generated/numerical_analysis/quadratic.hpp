#pragma once
#include "../basic_utility/binary_rational.hpp"
#include "common.hpp"
#include <vector>
namespace cp::numerical {
struct QuadraticRoots {
    Status status = Status::converged;
    bool all_real = false;
    std::vector<long double> roots; // ascending, a double root occurs once
};
inline QuadraticRoots quadratic_roots(long double a, long double b, long double c) {
    for (auto x : {a, b, c})
        detail::input(x);
    QuadraticRoots out;
    try {
        if (!a) {
            if (!b)
                out.all_real = !c;
            else
                out.roots.push_back(detail::checked(-c / b));
            return out;
        }
        if (!c) {
            const auto other = detail::checked(-b / a);
            out.roots = other == 0 ? std::vector<long double>{0}
                                   : std::vector<long double>{std::min(0.L, other), std::max(0.L, other)};
            return out;
        }
        const auto exact_a = rational_from_binary(a), exact_b = rational_from_binary(b),
                   exact_c = rational_from_binary(c);
        const auto discriminant = exact_b * exact_b - BigRational(4) * exact_a * exact_c;
        if (discriminant < 0)
            return out;
        if (!discriminant) {
            out.roots.push_back(rational_to_long_double(-exact_b / (BigRational(2) * exact_a)));
            return out;
        }
        const auto scale = std::max({std::abs(a), std::abs(b), std::abs(c)});
        const auto aa = a / scale, bb = b / scale, cc = c / scale;
        if (!aa || (b && !bb) || (c && !cc))
            throw detail::Stop{Status::non_finite};
        const auto exact_scale = rational_from_binary(scale);
        const auto d = rational_to_long_double(discriminant / (exact_scale * exact_scale));
        if (!d)
            throw detail::Stop{Status::non_finite};
        {
            const auto q = -(bb + std::copysign(std::sqrt(d), bb)) / 2;
            const auto x = detail::checked(q / aa), y = detail::checked(cc / q);
            out.roots = {std::min(x, y), std::max(x, y)};
        }
    } catch (const std::overflow_error &) {
        out.status = Status::non_finite;
        out.roots.clear();
    } catch (detail::Stop s) {
        out.status = s.status;
        out.roots.clear();
    }
    return out;
}
} // namespace cp::numerical
