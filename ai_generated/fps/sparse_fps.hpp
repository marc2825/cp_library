#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>
#include "detail/common.hpp"

namespace cp {
using SparseFPS = std::vector<std::pair<std::size_t, int>>;
namespace fps_detail {
template <int Mod> SparseFPS sparse_terms(const SparseFPS& input, std::size_t n) {
    std::map<std::size_t, int> coefficients;
    for (auto [degree, value] : input) if (degree < n) coefficients[degree] = Field<Mod>::add(coefficients[degree], Field<Mod>::norm(value));
    SparseFPS terms;
    for (auto [degree, value] : coefficients) if (value) terms.emplace_back(degree, value);
    return terms;
}
inline void sparse_length(std::size_t n) {
    if (n > std::vector<int>().max_size()) throw std::length_error("sparse FPS output too large");
}
} // namespace fps_detail
// 重複次数を加算、順序は任意。出力はdense、O(nk+k log k)。NTT不要。
template <int Mod = 998244353> std::vector<int> sparse_fps_inv(const SparseFPS& input, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::sparse_length(n); if (!n) return {};
    const auto terms = fps_detail::sparse_terms<Mod>(input, n);
    if (terms.empty() || terms[0].first) throw std::domain_error("sparse inverse needs nonzero constant");
    std::vector<int> result(n); result[0] = F::inv(terms[0].second);
    for (std::size_t t = 1; t < n; ++t) {
        int sum = 0;
        for (std::size_t j = 1; j < terms.size() && terms[j].first <= t; ++j) sum = F::add(sum, F::mul(terms[j].second, result[t - terms[j].first]));
        result[t] = F::sub(0, F::mul(sum, result[0]));
    }
    return result;
}
template <int Mod = 998244353> std::vector<int> sparse_fps_exp(const SparseFPS& input, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::sparse_length(n); if (!n) return {};
    const auto terms = fps_detail::sparse_terms<Mod>(input, n);
    if (!terms.empty() && !terms[0].first) throw std::domain_error("sparse exp needs zero constant");
    const auto inverses = fps_detail::inverses<Mod>(n - 1);
    std::vector<int> result(n); result[0] = 1;
    for (std::size_t t = 1; t < n; ++t) {
        int sum = 0;
        for (auto [degree, value] : terms) { if (degree > t) break; sum = F::add(sum, F::mul(F::mul(static_cast<int>(degree % Mod), value), result[t - degree])); }
        result[t] = F::mul(sum, inverses[t]);
    }
    return result;
}
template <int Mod = 998244353> std::vector<int> sparse_fps_log(const SparseFPS& input, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::sparse_length(n); if (!n) return {};
    const auto terms = fps_detail::sparse_terms<Mod>(input, n);
    if (terms.empty() || terms[0] != std::make_pair(std::size_t(0), 1)) throw std::domain_error("sparse log needs constant one");
    const auto inverse = sparse_fps_inv<Mod>(terms, n), inverses = fps_detail::inverses<Mod>(n - 1);
    std::vector<int> result(n);
    for (std::size_t t = 1; t < n; ++t) {
        int sum = 0;
        for (std::size_t j = 1; j < terms.size() && terms[j].first <= t; ++j)
            sum = F::add(sum, F::mul(F::mul(static_cast<int>(terms[j].first % Mod), terms[j].second), inverse[t - terms[j].first]));
        result[t] = F::mul(sum, inverses[t]);
    }
    return result;
}
template <int Mod = 998244353> std::vector<int> sparse_fps_pow(const SparseFPS& input, std::uint64_t exponent, std::size_t n) {
    using F = fps_detail::Field<Mod>;
    fps_detail::sparse_length(n); if (!n) return {};
    std::vector<int> result(n);
    if (!exponent) { result[0] = 1; return result; }
    const auto terms = fps_detail::sparse_terms<Mod>(input, n);
    if (terms.empty()) return result;
    const auto first = terms[0].first;
    if (first && exponent > (n - 1) / first) return result;
    const auto shift = first * static_cast<std::size_t>(exponent), length = n - shift;
    const auto inverses = fps_detail::inverses<Mod>(length - 1);
    const int a0_inverse = F::inv(terms[0].second), k = static_cast<int>(exponent % Mod);
    result[shift] = F::power(terms[0].second, exponent);
    for (std::size_t t = 1; t < length; ++t) {
        int sum = 0;
        for (std::size_t j = 1; j < terms.size() && terms[j].first - first <= t; ++j) {
            const auto degree = terms[j].first - first;
            const int factor = F::sub(F::mul(F::add(k, 1), static_cast<int>(degree % Mod)), static_cast<int>(t % Mod));
            sum = F::add(sum, F::mul(F::mul(factor, terms[j].second), result[shift + t - degree]));
        }
        result[shift + t] = F::mul(F::mul(sum, a0_inverse), inverses[t]);
    }
    return result;
}
} // namespace cp
