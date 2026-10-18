#pragma once
#include <cstddef>
#include <vector>
#include "../fps/inv.hpp"
#include "../fps/polynomial.hpp"

namespace cp {
// P/Q mod x^n。Q[0]!=0。n=0は分母条件を検査せず空。
template <int Mod = 998244353, int Root = 3> std::vector<int> generating_function_coefficients(
    const std::vector<int>& p, const std::vector<int>& q, std::size_t n) {
    fps_detail::check_precision<Mod, Root>(n);
    if (!n) return {};
    return fps_detail::product_prefix<Mod, Root>(p, fps_inv<Mod, Root>(q, n), n);
}
// 有限多項式の積の[x^degree]。空の因子列は1。
template <int Mod = 998244353, int Root = 3> int product_coefficient(
    const std::vector<std::vector<int>>& factors, std::size_t degree) {
    if (degree >= fps_max_size<Mod, Root>()) throw std::length_error("product coefficient exceeds FPS capacity");
    std::vector<std::vector<int>> level;
    for (const auto& f : factors) level.push_back(fps_truncate<Mod>(f, std::min(f.size(), degree + 1)));
    if (level.empty()) return degree ? 0 : 1;
    while (level.size() > 1) {
        std::vector<std::vector<int>> next;
        for (std::size_t i = 0; i < level.size(); i += 2) {
            if (i + 1 == level.size()) next.push_back(std::move(level[i]));
            else {
                auto value = fps_detail::multiply<Mod, Root>(level[i], level[i + 1]);
                if (value.size() > degree + 1) value.resize(degree + 1);
                next.push_back(std::move(value));
            }
        }
        level = std::move(next);
    }
    return degree < level[0].size() ? level[0][degree] : 0;
}
} // namespace cp
