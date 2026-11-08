#pragma once
#include <algorithm>
#include <cstddef>
#include <vector>
#include "polynomial.hpp"

namespace cp {
// 有限多項式f(g(x)) mod x^n。g[0]!=0も可（fの高次を捨てない）。
// baby-step/giant-step、O(sqrt(d) M(n)+nd)時間、O(n sqrt(d))領域。
template <int Mod = 998244353, int Root = 3>
std::vector<int> fps_compose(const std::vector<int>& f, const std::vector<int>& g, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::check_precision<Mod, Root>(n);
    if (!n) return {};
    if (f.empty()) return std::vector<int>(n);
    std::size_t block = 1;
    while (block < f.size() / block) ++block;
    const auto base = fps_truncate<Mod>(g, n);
    std::vector<std::vector<int>> powers(block + 1, std::vector<int>(n)); powers[0][0] = 1;
    for (std::size_t i = 1; i <= block; ++i) powers[i] = fps_detail::product_prefix<Mod, Root>(powers[i - 1], base, n);
    std::vector<int> result(n);
    const auto blocks = (f.size() - 1) / block + 1;
    for (std::size_t index = blocks; index-- > 0;) {
        result = fps_detail::product_prefix<Mod, Root>(result, powers[block], n);
        for (std::size_t j = 0; j < block && j < f.size() - index * block; ++j) {
            const int coefficient = F::norm(f[index * block + j]);
            for (std::size_t k = 0; k < n; ++k) result[k] = F::add(result[k], F::mul(coefficient, powers[j][k]));
        }
    }
    return result;
}
} // namespace cp
