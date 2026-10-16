#pragma once
#include <cstddef>
#include <vector>
#include "inv.hpp"
#include "polynomial.hpp"

namespace cp {
// log(a)=integral(a'/a) mod x^n。n>0ならa[0]=1。定数項0。
template <int Mod = 998244353, int Root = 3> std::vector<int> fps_log(const std::vector<int>& a, std::size_t n) {
    fps_detail::check_precision<Mod, Root>(n);
    if (!n) return {};
    if (a.empty() || fps_detail::Field<Mod>::norm(a[0]) != 1) throw std::domain_error("FPS logarithm requires constant one");
    auto d = polynomial_derivative<Mod>(fps_detail::prefix<Mod>(a, n));
    return polynomial_integral<Mod>(fps_detail::product_prefix<Mod, Root>(d, fps_inv<Mod, Root>(a, n), n - 1));
}
} // namespace cp
