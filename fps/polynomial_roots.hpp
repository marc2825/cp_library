#pragma once
#include "polynomial_factorization.hpp"
namespace cp {
// Distinct roots, sorted. Extract gcd(f,x^p-x); does not factor non-linear parts or scan the field.
template <int Mod = 998244353>
std::vector<int> polynomial_roots(const std::vector<int> &input, std::uint64_t seed = 0) {
    using A = finite_polynomial_detail::Algebra<Mod>;
    auto f = A::monic(input);
    if (f.empty())
        throw std::domain_error("zero polynomial has every field element as a root");
    if (f.size() == 1)
        return {};
    auto linear = A::gcd(f, A::sub(A::power({0, 1}, Mod, f), {0, 1}));
    std::vector<std::vector<int>> parts;
    if (linear.size() > 1)
        finite_polynomial_detail::split_equal_degree<Mod>(linear, 1, seed, parts);
    std::vector<int> roots;
    for (const auto &p : parts)
        roots.push_back(A::F::sub(0, p[0]));
    std::sort(roots.begin(), roots.end());
    return roots;
}
} // namespace cp
