#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "detail/common.hpp"
#include "kitamasa.hpp"

namespace cp {
// 観測列を説明する最短次数のc。s[t]=Σc[j]*s[t-1-j] (t>=c.size())。
// 素数法。空／全0は空、末尾0の係数は次数を保つため除去しない。
template <int Mod = 998244353> std::vector<int> berlekamp_massey(const std::vector<int>& samples) {
    static_assert(linear_recurrence_detail::prime(Mod), "Berlekamp-Massey requires a prime modulus");
    using F = linear_recurrence_detail::Arithmetic<Mod>;
    const auto s = linear_recurrence_detail::normalize<Mod>(samples);
    std::vector<int> connection{1}, previous{1};
    std::size_t length = 0, shift = 1;
    int last_discrepancy = 1;
    for (std::size_t index = 0; index < s.size(); ++index) {
        int discrepancy = s[index];
        for (std::size_t j = 1; j <= length; ++j) discrepancy = F::add(discrepancy, F::mul(connection[j], s[index - j]));
        if (!discrepancy) { ++shift; continue; }
        const auto saved = connection;
        const int scale = F::mul(discrepancy, F::inverse(last_discrepancy));
        if (shift > connection.max_size() - previous.size()) throw std::length_error("BM connection polynomial is too large");
        connection.resize(std::max(connection.size(), shift + previous.size()));
        for (std::size_t j = 0; j < previous.size(); ++j) connection[j + shift] = F::sub(connection[j + shift], F::mul(scale, previous[j]));
        if (length <= index / 2) {
            length = index + 1 - length; previous = saved; last_discrepancy = discrepancy; shift = 1;
        } else ++shift;
    }
    std::vector<int> c(length);
    for (std::size_t j = 0; j < length; ++j) c[j] = F::sub(0, connection[j + 1]);
    return c;
}
// BMが選ぶ漸化式による延長。未知の将来を保証するには真の次数の上限と十分な観測が必要。
// 空／全0は零列として延長。観測範囲内はその値を返す。
template <int Mod = 998244353> int linear_recurrence_from_prefix(const std::vector<int>& samples, std::uint64_t n) {
    static_assert(linear_recurrence_detail::prime(Mod), "recurrence inference requires a prime modulus");
    if (n < samples.size()) return linear_recurrence_detail::Arithmetic<Mod>::norm(samples[static_cast<std::size_t>(n)]);
    const auto c = berlekamp_massey<Mod>(samples);
    if (c.empty()) return 0;
    return kitamasa<Mod>(std::vector<int>(samples.begin(), samples.begin() + c.size()), c, n);
}
} // namespace cp
