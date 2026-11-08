#pragma once
#include <algorithm>
#include <cstddef>
#include <vector>
#include "detail/common.hpp"

namespace cp {
// 昇冪順の係数。全て入力不変、結果は[0,Mod)。末尾0はtrim以外では保持。
template <int Mod = 998244353> std::vector<int> polynomial_add(const std::vector<int>& a, const std::vector<int>& b) {
    auto c = fps_detail::prefix<Mod>(a, std::max(a.size(), b.size()));
    for (std::size_t i = 0; i < b.size(); ++i) c[i] = fps_detail::Field<Mod>::add(c[i], fps_detail::Field<Mod>::norm(b[i]));
    return c;
}
template <int Mod = 998244353> std::vector<int> polynomial_subtract(const std::vector<int>& a, const std::vector<int>& b) {
    auto c = fps_detail::prefix<Mod>(a, std::max(a.size(), b.size()));
    for (std::size_t i = 0; i < b.size(); ++i) c[i] = fps_detail::Field<Mod>::sub(c[i], fps_detail::Field<Mod>::norm(b[i]));
    return c;
}
template <int Mod = 998244353, int Root = 3> std::vector<int> polynomial_multiply(const std::vector<int>& a, const std::vector<int>& b) {
    return fps_detail::multiply<Mod, Root>(a, b);
}
template <int Mod = 998244353> std::vector<int> polynomial_derivative(const std::vector<int>& a) {
    std::vector<int> b(a.empty() ? 0 : a.size() - 1);
    for (std::size_t i = 1; i < a.size(); ++i)
        b[i - 1] = fps_detail::Field<Mod>::mul(fps_detail::Field<Mod>::norm(a[i]), static_cast<int>(i % Mod));
    return b;
}
// 積分定数は0。a.size()<Modが必要。
template <int Mod = 998244353> std::vector<int> polynomial_integral(const std::vector<int>& a) {
    auto b = fps_detail::inverses<Mod>(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) b[i + 1] = fps_detail::Field<Mod>::mul(fps_detail::Field<Mod>::norm(a[i]), b[i + 1]);
    return b;
}
// a mod x^n。正確にn項を返し、足りない項は0で埋める。
template <int Mod = 998244353> std::vector<int> fps_truncate(const std::vector<int>& a, std::size_t n) {
    return fps_detail::prefix<Mod>(a, n);
}
template <int Mod = 998244353> std::vector<int> polynomial_trim(const std::vector<int>& a) {
    auto b = fps_detail::prefix<Mod>(a, a.size()); fps_detail::trim(b); return b;
}
template <int Mod = 998244353> int polynomial_evaluate(const std::vector<int>& a, long long x) {
    using F = fps_detail::Field<Mod>;
    int v = 0, t = F::norm(x);
    for (std::size_t i = a.size(); i-- > 0;) v = F::add(F::mul(v, t), F::norm(a[i]));
    return v;
}
} // namespace cp
