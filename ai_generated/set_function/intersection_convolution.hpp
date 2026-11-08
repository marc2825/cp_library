#pragma once
#include "fast_subset_convolution.hpp"
namespace cp {
// result[k][S] = sum_{A union B=S, |A intersection B|=k} f[A]g[B].
// Ranked zeta, rank-polynomial product, Mobius: O(b^2 2^b) time/space O(b 2^b).
template <int Mod = 998244353>
std::vector<std::vector<int>> intersection_size_convolution(const std::vector<int> &f,
                                                            const std::vector<int> &g) {
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    set_function_detail::validate_size(f.size());
    if (f.size() != g.size())
        throw std::invalid_argument("intersection convolution sizes differ");
    std::size_t bits = 0;
    for (auto s = f.size(); s > 1; s /= 2)
        ++bits;
    std::vector<std::size_t> rank(f.size());
    for (std::size_t s = 1; s < f.size(); ++s)
        rank[s] = rank[s / 2] + (s & 1);
    std::vector<std::vector<int>> a(bits + 1, std::vector<int>(f.size())), b = a, result = a;
    for (std::size_t s = 0; s < f.size(); ++s) {
        a[rank[s]][s] = F::norm(f[s]);
        b[rank[s]][s] = F::norm(g[s]);
    }
    for (std::size_t bit = 1; bit < f.size(); bit *= 2)
        for (std::size_t s = 0; s < f.size(); ++s)
            if (s & bit)
                for (std::size_t k = 0; k <= bits; ++k) {
                    a[k][s] = F::add(a[k][s], a[k][s ^ bit]);
                    b[k][s] = F::add(b[k][s], b[k][s ^ bit]);
                }
    for (std::size_t k = 0; k <= 2 * bits; ++k) {
        std::vector<int> row(f.size());
        for (std::size_t i = k > bits ? k - bits : 0; i <= std::min(k, bits); ++i)
            for (std::size_t s = 0; s < f.size(); ++s)
                row[s] = F::add(row[s], F::mul(a[i][s], b[k - i][s]));
        for (std::size_t bit = 1; bit < f.size(); bit *= 2)
            for (std::size_t s = 0; s < f.size(); ++s)
                if (s & bit)
                    row[s] = F::sub(row[s], row[s ^ bit]);
        for (std::size_t s = 0; s < f.size(); ++s)
            if (k >= rank[s] && k - rank[s] <= bits)
                result[k - rank[s]][s] = row[s];
    }
    return result;
}
} // namespace cp
