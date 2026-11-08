#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../fps/inv.hpp"
#include "../fps/log.hpp"
#include "../fps/exp.hpp"

namespace cp {
namespace linear_recurrence_detail {
template <int Mod, int Root> void egf_precision(std::size_t n) {
    fps_detail::check_precision<Mod, Root>(n);
    if (n > static_cast<std::size_t>(Mod)) throw std::domain_error("EGF factorial is not invertible");
}
} // namespace linear_recurrence_detail
// EGF配列aは sum a[k]*x^k/k! を表す。ordinaryは実際のx^k係数。
template <int Mod = 998244353> std::vector<int> egf_to_ordinary(const std::vector<int>& a) {
    using F = fps_detail::Field<Mod>;
    if (a.size() > static_cast<std::size_t>(Mod)) throw std::domain_error("EGF factorial is not invertible");
    if (a.empty()) return {};
    const auto inverses = fps_detail::inverses<Mod>(a.size() - 1);
    std::vector<int> result(a.size()); int scale = 1;
    for (std::size_t i = 0; i < a.size(); ++i) { if (i) scale = F::mul(scale, inverses[i]); result[i] = F::mul(F::norm(a[i]), scale); }
    return result;
}
template <int Mod = 998244353> std::vector<int> egf_from_ordinary(const std::vector<int>& a) {
    using F = fps_detail::Field<Mod>;
    if (a.size() > static_cast<std::size_t>(Mod)) throw std::domain_error("EGF factorial is not invertible");
    std::vector<int> result(a.size()); int scale = 1;
    for (std::size_t i = 0; i < a.size(); ++i) { if (i) scale = F::mul(scale, static_cast<int>(i)); result[i] = F::mul(F::norm(a[i]), scale); }
    return result;
}
template <int Mod = 998244353, int Root = 3> std::vector<int> egf_multiply(const std::vector<int>& a, const std::vector<int>& b, std::size_t n) {
    linear_recurrence_detail::egf_precision<Mod, Root>(n);
    return egf_from_ordinary<Mod>(fps_detail::product_prefix<Mod, Root>(egf_to_ordinary<Mod>(fps_truncate<Mod>(a, n)), egf_to_ordinary<Mod>(fps_truncate<Mod>(b, n)), n));
}
template <int Mod = 998244353, int Root = 3> std::vector<int> egf_inv(const std::vector<int>& a, std::size_t n) {
    linear_recurrence_detail::egf_precision<Mod, Root>(n);
    return egf_from_ordinary<Mod>(fps_inv<Mod, Root>(egf_to_ordinary<Mod>(fps_truncate<Mod>(a, n)), n));
}
template <int Mod = 998244353, int Root = 3> std::vector<int> egf_log(const std::vector<int>& a, std::size_t n) {
    linear_recurrence_detail::egf_precision<Mod, Root>(n);
    return egf_from_ordinary<Mod>(fps_log<Mod, Root>(egf_to_ordinary<Mod>(fps_truncate<Mod>(a, n)), n));
}
template <int Mod = 998244353, int Root = 3> std::vector<int> egf_exp(const std::vector<int>& a, std::size_t n) {
    linear_recurrence_detail::egf_precision<Mod, Root>(n);
    return egf_from_ordinary<Mod>(fps_exp<Mod, Root>(egf_to_ordinary<Mod>(fps_truncate<Mod>(a, n)), n));
}
} // namespace cp
