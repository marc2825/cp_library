#pragma once
#include <optional>
#include <vector>
#include "quotient_prefix.hpp"

namespace cp {
// Commutative ring. inverse(f(1)) -> optional<T>; nonunit -> nullopt. Index zero ignored.
template<class T, class Inverse>
std::optional<std::vector<T>> dirichlet_inverse(const std::vector<T>& f, Inverse inverse) {
    using namespace quotient_detail;
    std::vector<T> result(f.size(), T(0));
    if (f.size() <= 1) return result;
    const auto unit = inverse(f[1]);
    if (!unit) return std::nullopt;
    result[1] = T(1);
    for (std::size_t i = 1; i < f.size(); ++i) {
        result[i] = mul(result[i], *unit);
        for (std::size_t j = 2; j <= (f.size() - 1) / i; ++j) result[i * j] = sub(result[i * j], mul(result[i], f[j]));
    }
    return result;
}
template<class T, class Inverse>
std::optional<QuotientPrefix<T>> dirichlet_prefix_inverse(const QuotientPrefix<T>& f, Inverse inverse) {
    using namespace quotient_detail;
    QuotientPrefix<T> result(f.limit(), [](auto) { return T(0); }, f.root());
    if (!f.limit()) return result;
    const auto unit = inverse(f.sum(1));
    if (!unit) return std::nullopt;
    for (auto it = f.quotients().rbegin(); it != f.quotients().rend(); ++it) {
        const auto x = *it;
        T value = T(1);
        for (std::uint64_t left = 2; left <= x;) {
            const auto q = x / left, right = x / q;
            value = sub(value, mul(sub(f.sum(right), f.sum(left - 1)), result.sum(q)));
            if (right == x) break;
            left = right + 1;
        }
        result.set(x, mul(value, *unit));
    }
    return result;
}
} // namespace cp
