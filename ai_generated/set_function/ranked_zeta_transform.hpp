#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "subset_zeta_transform.hpp"
#include "subset_mobius_transform.hpp"

namespace cp {
// 戻り値[rank][mask]=sum_{T subset mask, |T|=rank}f[T]。
// 正の2冪長、O(N log^2 N)時間・O(N log N)領域。
template <class T> std::vector<std::vector<T>> ranked_zeta_transform(const std::vector<T>& f) {
    set_function_detail::validate_size(f.size());
    std::size_t bits = 0; for (auto n = f.size(); n > 1; n >>= 1) ++bits;
    std::vector<std::vector<T>> result(bits + 1, std::vector<T>(f.size(), T(0)));
    std::vector<std::size_t> rank(f.size());
    for (std::size_t s = 1; s < f.size(); ++s) rank[s] = rank[s >> 1] + (s & 1);
    for (std::size_t s = 0; s < f.size(); ++s) result[rank[s]][s] = f[s];
    for (auto& row : result) subset_zeta_transform(row);
    return result;
}
// rank層ごとの逆変換。rank数と全行の長さを変更前に検査。一般のrank表も可。
// 逆変換後の[|S|][S]で元のf[S]を取得。docs/set_function/ranked_zeta_transform.md
template <class T> void ranked_mobius_transform(std::vector<std::vector<T>>& f) {
    if (f.empty()) throw std::invalid_argument("ranked table must have a rank-zero row");
    set_function_detail::validate_size(f.front().size());
    std::size_t bits = 0; for (auto n = f.front().size(); n > 1; n >>= 1) ++bits;
    if (f.size() != bits + 1) throw std::invalid_argument("ranked table rank count mismatch");
    for (const auto& row : f) if (row.size() != f.front().size()) throw std::invalid_argument("ranked table row size mismatch");
    for (auto& row : f) subset_mobius_transform(row);
}
} // namespace cp
